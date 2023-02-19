#include "main.h"

// Global Variables
bool isPTO = false; // false is able to winch
bool PTOenabled = true;
bool isBack = false;
bool halfShot = false;
int PTOCount = 32;
int target;

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
    pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(1, 8313, 12919, 10616, -2185, -343, -1264, 3.000, 0);
	pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(2, -2341, -1281, -1811, 6097, 8227, 7162, 3.600, 0);
    vision_sensor.set_signature(1, &RED_SIG);
	vision_sensor.set_signature(2, &BLUE_SIG);
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
		Skills();
	} 
}

void opcontrol() {
	leftDrive.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightDrive.setBrakeMode(AbstractMotor::brakeMode::coast);
	
	while (true) {
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
			PTO();
			winchGroup.moveVoltage(12000);
		}
		else if (master.getDigital(ControllerDigital::L2) && !isPTO) { // Winch
			winchGroup.setBrakeMode(AbstractMotor::brakeMode::hold);
			winchGroup.moveVoltage(12000);
		} 
		else if (master.getDigital(ControllerDigital::L1) && !isPTO && isBack) { // Unwinch
			target = rotation_sensor.get() + 200;
			halfShot = true;
			winchGroup.moveVoltage(-6000);
			isBack = false;
			pros::delay(300);
		}
		if (rotation_sensor.get() >= target && halfShot) {
			winchGroup.moveVoltage(1200);
			halfShot = false;
		}
		if (limit_switch.get_value() == 1 && !isBack) {
			winchGroup.moveVoltage(1200);
			isBack = true;
		}
		

		if (master.getDigital(ControllerDigital::up)) { // Aim Bot"?
			turnToGoal();
		}

		// Intake
		if ((master.getDigital(ControllerDigital::R1) && master.getDigital(ControllerDigital::R2))) {
			intake_roller.moveVoltage(0);
			pros::delay(200);
		}
		else if ((master.getDigital(ControllerDigital::R2) && limit_switch.get_value() == 1)) {
			intake_roller.moveVoltage(12000);
		}
		else if (master.getDigital(ControllerDigital::R1)) {
			intake_roller.moveVoltage(-12000);
		}
		// Intake
		if ((partner.getDigital(ControllerDigital::R1) && partner.getDigital(ControllerDigital::R2))) {
			intake_roller.moveVoltage(0);
			pros::delay(200);
		}
		else if ((partner.getDigital(ControllerDigital::R2) && limit_switch.get_value() == 1)) {
			intake_roller.moveVoltage(12000);
		}
		else if (partner.getDigital(ControllerDigital::R1)) {
			intake_roller.moveVoltage(-12000);
		}

		// Drive
		chassis->getModel()->tank(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightY));
		pros::delay(20);

	}
}
