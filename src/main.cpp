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
	arms::odom::reset({48, 120}, 90);
	while(true) {
		arms::chassis::tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		pros::delay(10);
	}
}

