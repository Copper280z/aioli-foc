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

// #include "./can.cpp"

// Motor specific parameters.
#define POLEPAIRS 7
#define RPHASE 5.3
#define MOTORKV 140

uint8_t useDFU = 0;
uint8_t pendingFrame = 0;
uint8_t sfocCmdStr;

PIDController haptic_pid = PIDController(
    2.0f,
    0.0f,
    0.05f,
    10000.0f,
    1.4f
);


// simpleFOC constructors
BLDCDriver3PWM driver = BLDCDriver3PWM(U_PWM, V_PWM, W_PWM, U_EN, V_EN, W_EN);
BLDCMotor motor = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
MagneticSensorMT6701SSI enc = MagneticSensorMT6701SSI(ENC_CS);
// SmoothingSensor enc = SmoothingSensor(encoder, motor);
Commander commander = Commander(SerialUSB);
HapticInterface haptic = HapticInterface(&motor, &haptic_pid);



uint8_t configureFOC(void);
uint8_t configureCAN(void);
uint8_t configureDFU(void);

void doMotor(char *cmd)
{
	commander.motor(&motor, cmd);
}

void setup()
{
	pinMode(USER_LED, OUTPUT);
	pinMode(USER_BUTTON, INPUT);
	
	if (digitalRead(USER_BUTTON) == HIGH){
		jump_to_bootloader();
	}

	SerialUSB.begin();

	Serial.println(configureFOC() == 1 ? "SFOC successfully init." : "SFOC failed to init.");
	Serial.println(configureCAN() == 1 ? "CAN successfully init."  : "CAN failed to init.");
	Serial.println(configureDFU() == 1 ? "DFU successfully init."  : "DFU failed to init.");
}

void loop()
{
	// motor.loopFOC();
	// motor.move();
	haptic.haptic_loop();
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
}

uint8_t configureFOC(){
	commander.add('M', doMotor, "motor");
	commander.verbose = VerboseMode::machine_readable;
	
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

	motor.sensor_offset = 2.43;
	motor.sensor_direction = Direction::CCW;

	motor.linkSensor(&enc);
	motor.linkDriver(&driver);
	motor.init();

	motor.target = 0;

	haptic.haptic_config->sfoc_voltage_control = true;
	haptic.haptic_config->last_pos=1;
	haptic.haptic_config->current_pos=1;
	haptic.haptic_config->total_pos=42;
	haptic.haptic_config->end_pos=41;
	haptic.haptic_config->start_pos=1;
	haptic.haptic_config->distance_space=10;
	haptic.haptic_config->distance_pos = haptic.haptic_config->distance_space * _PI / 180;
	

	motor.initFOC();
	haptic.init();


	return 0;
}

uint8_t configureCAN(){
	return 0;
}

uint8_t configureDFU(){
	return 1;
}