#include "main.h"


pros::Controller master(pros::E_CONTROLLER_MASTER);


void piston(pros::ADIDigitalOut piston, bool intially_extended, bool extend) {
	if(intially_extended) {
		if(extend) {
			piston.set_value(false);
		}
		else {
			piston.set_value(true);
		}
	}
	else {
		if(extend) {
			piston.set_value(true);
		}
		else {
			piston.set_value(false);
		}
	}
}


void initialize() {
	arms::init();
}


void disabled() {}


void competition_initialize() {}


void autonomous() {}


void opcontrol() {
	arms::odom::reset({48, 120}, -90);
	arms::chassis::setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	PathFollower pathfollower(0.37465);
	pathfollower.createConstraints("test", 0.275666666666, 9, 18);
	//pathfollower.createPath("test", "test_path", {{48, 120, -90}, {72, 96, -90}, {72, 24, 0}, {120, 48, -90}, {120, 48, -180}});
	pathfollower.createPath("test", "test_path", {{48, 120, -90}, {48, 72, -90}});
	pathfollower.printPath("test_path");



	while(true) {
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			pathfollower.followPath("test_path");
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			pathfollower.followPathRamsete("test_path");
		}
		arms::chassis::tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		pros::delay(10);
	}

}

