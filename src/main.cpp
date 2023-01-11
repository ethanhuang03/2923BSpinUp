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
	PathFollower pathfollower(0.37465);
	pathfollower.createConstraints("fast", 1.3, 2, 2);
	pathfollower.createConstraints("slow", 0.5, 2, 2);
	pathfollower.createPath("fast", "path_fast", {{48, 120, 0}, {72, 96, 180}, {72, 24, 90}, {120, 48, 0}, {120, 48, -90}});
	pathfollower.createPath("slow", "path_slow", {{48, 120, 0}, {72, 96, 180}, {72, 24, 90}, {120, 48, 0}, {120, 48, -90}});
	pathfollower.followPath("path_fast");

	/*
	arms::odom::reset({48, 120}, 90);
	while(true) {
		arms::chassis::tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		pros::delay(10);
	}
	*/
}

