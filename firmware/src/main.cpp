#include <Arduino.h>
#include <Spi.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#include "encoders/smoothing/SmoothingSensor.h"
#include "stm32g431xx.h"
#include "drv_reset.h"
#include "aioli-board.h"
#include <haptic.h>

#include "usbd_hid_composite_if.h"

// #include "./can.cpp"

// Motor specific parameters.
#define POLEPAIRS 7
#define RPHASE 5.3
#define MOTORKV 140

uint8_t useDFU = 0;
uint8_t pendingFrame = 0;
uint8_t sfocCmdStr;

// PIDController haptic_pid = PIDController(
//     2.0f,
//     0.0f,
//     0.05f,
//     10000.0f,
//     1.4f
// );

// simpleFOC constructors
BLDCDriver3PWM driver = BLDCDriver3PWM(U_PWM, V_PWM, W_PWM, U_EN, V_EN, W_EN);
BLDCMotor motor = BLDCMotor(POLEPAIRS, RPHASE);
MagneticSensorMT6701SSI enc = MagneticSensorMT6701SSI(ENC_CS);
// SmoothingSensor enc = SmoothingSensor(encoder, motor);
// #ifdef USBD_USE_CDC
Commander commander = Commander(SerialUSB);
// #endif
// HapticInterface haptic = HapticInterface(&motor, &haptic_pid);

class Texture {
public:
	// defined in a normalized coordinate system.
	// angle of 0.0 is the start of the texture
	// angle of 1.0 is the end of the texture
	virtual float get_torque(float angle, float velocity, float acceleration) = 0;
};
class OneWayRatchet: public Texture {
public:
	float ramp_start = 0.1;
	float ramp_end = 0.5f;
	float ramp_peak_force = 0.5;
	float hysteresis = 0.25f;
	float central_deadzone = 0.05;

	float centering_spring_rate = 0.3f;
	float damping = 0.0005f;
	float inertia = 0.0001f;

	float get_torque(float angle, float velocity, float acceleration) final {
		float output_torque = 0;
		if (angle > central_deadzone)
			output_torque += angle * centering_spring_rate;
		output_torque += damping * velocity;
		output_torque += inertia * acceleration;

		// check to see if we've fallen off the ramp
		if (angle > ramp_end) {
			ramp_engaged = false;
		} // check to see if we've reset back onto the ramp
		else if (!ramp_engaged & angle < (ramp_end-hysteresis)) {
			ramp_engaged = true;
		}

		if (ramp_engaged & angle > ramp_start) {
			float ramp_slope = ramp_peak_force / (ramp_end - ramp_start);
			output_torque += (angle - ramp_start) * ramp_slope;
		}
		return output_torque;
	};
private:
	bool ramp_engaged = false;
};

class Renderer {
public:
	float epsilon = 1e-8;

	// rather unique_ptr or array/vector of textures
	Texture* tex;
	float texture_start = 0.13f;
	float texture_end = 0.52f + 0.13f;
	float endstop_gain = 5.0f;

	Renderer() : velocity_lpf(LowPassFilter(1.0/(30*_2PI))) , accel_lpf(LowPassFilter(1.0/(20*_2PI))) {}

	float get_torque(float current_position) {
		uint32_t current_time = micros();
		float position_change = current_position - previous_position;
		if (fabsf(position_change) < epsilon) return previous_torque;

		float delta_time_position = (float) (current_time - previous_position_time) * 1e-6f;
		float current_velocity = velocity_lpf(position_change / delta_time_position); // rad/sec
		
		// microseconds
		float current_velocity_time = previous_position_time + (float)(current_time - previous_position_time) / 2.0f;
		float delta_time_velocity = (current_velocity_time - previous_velocity_time) * 1e-6f;
		float velocity_change = current_velocity - previous_velocity; 

		float current_acceleration = accel_lpf(velocity_change / delta_time_velocity); // rad/sec^2

		float texture_length = texture_end - texture_start;
		float norm_position = (current_position - texture_start) / texture_length;
		float norm_velocity = current_velocity / texture_length;
		float norm_acceleration = current_acceleration / texture_length;

		float position_sign = _sign(norm_position);
		norm_position = fabsf(norm_position);
		float output_torque = 0;

		output_torque += position_sign * get_endstop_torque(norm_position);
		norm_position = _constrain(norm_position, 0.0f, 1.0f);
		output_torque += position_sign * tex->get_torque(norm_position, position_sign*norm_velocity, position_sign*norm_acceleration);
		if (output_torque > 1.0f) output_torque = 1.0f;
		if (output_torque < -1.0f) output_torque = -1.0f;

		previous_position = current_position;
		previous_position_time = current_time;
		previous_velocity = current_velocity;
		previous_velocity_time = current_velocity_time;
		previous_acceleration = current_acceleration;
		previous_torque = output_torque;

		prev_norm_pos = norm_position;

		return output_torque;
	}

	float get_endstop_torque (float current_normed_position) {
		float endstop_torque = 0;
		if (current_normed_position > 1.0f) {
			float endstop_position_error = current_normed_position - 1.0f;
			endstop_torque = endstop_gain * endstop_position_error;
		}
		return endstop_torque;
	}

// private:
	float previous_position = 0.0f;
	uint32_t previous_position_time = 0;
	float previous_velocity = 0.0f;
	float previous_velocity_time = 0; // allowing decimal microseconds
	float previous_torque = 0.0f;
	LowPassFilter velocity_lpf;
	LowPassFilter accel_lpf;

	float prev_norm_pos = 0;
	float previous_acceleration = 0;
};

OneWayRatchet ratchet_texture;
Renderer texture_renderer;

uint8_t configureFOC(void);
uint8_t configureCAN(void);
uint8_t configureDFU(void);

void doMotor(char *cmd)
{
	#ifdef USBD_USE_CDC
	commander.motor(&motor, cmd);
	#endif
}

void setup()
{
	pinMode(USER_LED, OUTPUT);
	pinMode(USER_BUTTON, INPUT);
	
	if (digitalRead(USER_BUTTON) == HIGH){
		jump_to_bootloader();
	}
// #ifdef USBD_USE_CDC
	SerialUSB.begin();
// #endif
	texture_renderer.tex = &ratchet_texture;

	configureFOC();
	configureCAN();
	configureDFU() ;
	// HID_Composite_Init(HID_Interface::HID_KEYBOARD);
	// Serial.println(configureFOC() == 1 ? "SFOC successfully init." : "SFOC failed to init.");
	// Serial.println(configureCAN() == 1 ? "CAN successfully init."  : "CAN failed to init.");
	// Serial.println(configureDFU() == 1 ? "DFU successfully init."  : "DFU failed to init.");
}

void loop()
{
	motor.loopFOC();
	float position = enc.getAngle();
	float tex_torque = motor.current_limit * texture_renderer.get_torque(position);
	if (motor.controller == MotionControlType::torque){
		motor.move(tex_torque);
	} else {
		motor.move();
	}

	static uint32_t loop_counter = 0;
	
	if (loop_counter > 1000) {
		loop_counter = 0;
		SerialUSB.printf("pos: %6.3f, vel: %6.3f, acc: %9.3f, norm_pos: %6.3f, tex_trq: %6.3f\n", texture_renderer.previous_position, texture_renderer.previous_velocity, texture_renderer.previous_acceleration, texture_renderer.prev_norm_pos, tex_torque);
	}
	
	loop_counter+=1;

	// haptic.haptic_loop();
	commander.run();

	// if(pendingFrame){
	// 	commander.run((char*)sfocCmdStr);
	// 	pendingFrame = 0;
	// }
	// else{
	// 	commander.run();
	// }

	#ifdef HAS_MONITOR
	motor.monitor();
	#endif

	// TODO: if serial command inits keyboard/mouse config then do this
	// uint8_t HIDbuffer[8] = {0}; 
	// if(haptic.haptic_config->attract_angle > haptic.haptic_config->current_pos * haptic.haptic_config->distance_pos && haptic.haptic_config->current_pos < haptic.haptic_config->end_pos){
	// 	HIDbuffer[2] = KEY_VOLUMEUP;
    // } else if (haptic.haptic_config->attract_angle < haptic.haptic_config->current_pos * haptic.haptic_config->distance_pos && haptic.haptic_config->current_pos > haptic.haptic_config->start_pos){
	// 	HIDbuffer[2] = KEY_VOLUMEDOWN;
    // }
	// float angle_diff = haptic.haptic_config->attract_angle - haptic.haptic_config->last_attract_angle;
	// if (angle_diff > haptic.haptic_config->distance_pos*haptic.haptic_config->current_pos) {
	// 	// HID_Composite_keyboard_sendReport(HIDbuffer, 8);
	// } else 	if (angle_diff < -haptic.haptic_config->distance_pos*haptic.haptic_config->current_pos) {
	// 	HIDbuffer[2] = KEY_VOLUMEDOWN;
	// 	// HID_Composite_keyboard_sendReport(HIDbuffer, 8);
	// }
	// HID_Composite_keyboard_sendReport(HIDbuffer, 8);

}

uint8_t configureFOC(){
	// #ifdef USBD_USE_CDC
	commander.add('M', doMotor, "motor");
	// commander.verbose = VerboseMode::machine_readable;
	// #endif

	#ifdef SIMPLEFOC_STM_DEBUG
	SimpleFOCDebug::enable(&Serial);
	#endif

	// Encoder initialization.
	// Encoder on SPI1
	enc.init();

	// Driver initialization.
	driver.pwm_frequency = 32000;
	driver.voltage_power_supply = 5;
	driver.voltage_limit = 4.5;
	driver.init();

	// Motor PID parameters.
	// motor.PID_velocity.P = 0.2;
	// motor.PID_velocity.I = 3;
	// motor.PID_velocity.D = 0.002;
	motor.voltage_limit = 2.2;
	motor.PID_velocity.output_ramp = 1000;
	motor.LPF_velocity.Tf = 0.5; // 1/(6.28*250);
	motor.LPF_angle.Tf = 1/(100*_2PI); // try to avoid

	// Motor initialization.
	motor.voltage_sensor_align = 2;
	motor.current_limit = 0.5;
	motor.velocity_limit = 20;
	motor.controller = MotionControlType::torque;
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

	// motor.sensor_offset = 2.43;
	// motor.sensor_direction = Direction::CCW;

	motor.linkSensor(&enc);
	motor.linkDriver(&driver);
	motor.init();

	motor.target = 0;

	// haptic.haptic_config->sfoc_voltage_control = true;
	// haptic.haptic_config->last_pos=1;
	// haptic.haptic_config->current_pos=1;
	// haptic.haptic_config->total_pos=42;
	// haptic.haptic_config->end_pos=41;
	// haptic.haptic_config->start_pos=1;
	// haptic.haptic_config->distance_space=10;
	// haptic.haptic_config->distance_pos = haptic.haptic_config->distance_space * _PI / 180;
	

	motor.initFOC();
	// haptic.init();


	return 0;
}

uint8_t configureCAN(){
	return 0;
}

uint8_t configureDFU(){
	return 1;
}