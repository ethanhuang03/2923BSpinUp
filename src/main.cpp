#include "main.h"

// Global Variables
bool isPTO = false; // false is able to winch
bool PTOenabled = true;
bool isBack = false;
bool halfShot = false;
int PTOCount = 32;

// Utility Functions
void PTO() {
	PTO1.toggle();
	isPTO = !isPTO;
	if(isPTO) {
		winchGroup.setBrakeMode(AbstractMotor::brakeMode::coast);
	} else {
		winchGroup.setBrakeMode(AbstractMotor::brakeMode::hold);
	}
}

// Competition
void initialize() {
	imu.calibrate();
	selector::init();
	
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	leftDrive.setBrakeMode(AbstractMotor::brakeMode::brake);
    rightDrive.setBrakeMode(AbstractMotor::brakeMode::brake);
	if (selector::auton == 0) {
		Skills();
	} else if (selector::auton == 1) {
		Left();
	} else if (selector::auton == 2) {
		Right();
	} else if (selector::auton == 3) {
		AWP();
	} 
}

void opcontrol() {
	
	leftDrive.setBrakeMode(AbstractMotor::brakeMode::coast);
	rightDrive.setBrakeMode(AbstractMotor::brakeMode::coast);

	while (true) {
		// PTO Warning
		// Display how many shots left and when numnber of shots is 0, disable PTO
		master.clear();
		std::string display = "Shots Left: "+ PTOCount;
		master.setText(0, 0, display);
		if (PTOCount < 10 && PTOCount > 0){
			master.rumble("----");
			master.setText(1, 0, "Low");
		}
		else if (PTOCount == 0){
			master.setText(0, 0, "Empty");
			PTOenabled = false;
		}
		if (!PTOenabled && master.getDigital(ControllerDigital::Y)){ // Override Function because why not?
			PTOenabled = true;
		}

		// Expansion
		if (master.getDigital(ControllerDigital::X)){ // X for Xpansion
			expansion.toggle();
			pros::delay(200);
		}

		// PTO And Winch
		if (master.getDigital(ControllerDigital::L1) && master.getDigital(ControllerDigital::L2) && PTOenabled){
			winchGroup.moveVoltage(0);
			PTO();
			PTOCount -= 1;
			winchGroup.setBrakeMode(AbstractMotor::brakeMode::coast);
			isBack = false;
			pros::delay(200);
		}
		else if (master.getDigital(ControllerDigital::L2) && !isPTO) { // Winch
			winchGroup.setBrakeMode(AbstractMotor::brakeMode::hold);
			winchGroup.moveVoltage(12000);
		} 
		else if (master.getDigital(ControllerDigital::L1) && !isPTO && isBack) { // Unwinch
			rotation_sensor.reset();
			halfShot = true;
			winchGroup.moveVoltage(-6000);
			isBack = false;
			pros::delay(200);
		}
		if (rotation_sensor.get() >= 20 && halfShot) {
			winchGroup.moveVoltage(1200);
			halfShot = false;
		}
		if (limit_switch.get_value() == 1 && !isBack) {
			winchGroup.moveVoltage(1200);
			rotation_sensor.reset();
			isBack = true;
		}
		

		

		// Intake
		if (master.getDigital(ControllerDigital::R1) && master.getDigital(ControllerDigital::R2) || limit_switch.get_value() == 0) {
			intake_roller.moveVoltage(0);
			pros::delay(200);
		}
		else if (master.getDigital(ControllerDigital::R2) && limit_switch.get_value() == 1) {
			intake_roller.moveVoltage(12000);
		}
		else if (master.getDigital(ControllerDigital::R1)) {
			intake_roller.moveVoltage(-12000);
		}
		
		// Drive
		chassis->getModel()->tank(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightY));
		pros::delay(20);
	}
}
