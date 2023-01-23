#include "main.h"

// Global Variables
const double DEADBAND = 0.0500;
bool isPTO = false; // false is able to winch

// Utility Functions
void PTO() {
	PTO1.toggle();
	isPTO = !isPTO;
	if(isPTO) {
		drive = chassisPTO;
	} else {
		drive = chassis;
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
		LeftAWP();
	} else if (selector::auton == 4) {
		RightAWP();
	}

}

void opcontrol() {
	leftDrive.setBrakeMode(AbstractMotor::brakeMode::coast);
	rightDrive.setBrakeMode(AbstractMotor::brakeMode::coast);
	drive = chassis;

	while (true) {

		// PTO Toggle
		if (master.getDigital(ControllerDigital::A)) {
			PTO();
			pros::delay(200);
		}

		// Winch
		if (isPTO == false) {
			if (master.getDigital(ControllerDigital::L1)) {
				winchPTO.moveVoltage(12000);
			} else if (master.getDigital(ControllerDigital::L2)) {
				winchPTO.moveVoltage(-12000);
			} else {
				winchPTO.moveVoltage(0);
			}
		}

		if (master.getDigital(ControllerDigital::R1)) {
			intake_roller.moveVoltage(12000);
		}
		else if (master.getDigital(ControllerDigital::R2)) {
			intake_roller.moveVoltage(-12000);
		}
		else {
			intake_roller.moveVoltage(0);
		}

		// Drive
		drive->getModel()->curvature(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightX), DEADBAND);
		pros::delay(20);
	}
}
