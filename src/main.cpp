#include "main.h"


pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);

pros::Motor intake(1);
pros::Motor winch(2);

pros::ADIDigitalOut winch_pto('A');
pros::ADIDigitalOut expansion('B');


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


void fire() {}


void reload() {}


void aimbot() {}


void initialize() {
	arms::init();
}


void disabled() {}


void competition_initialize() {}


void autonomous() {}


void opcontrol() {
	while(true) {
		arms::chassis::tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		pros::delay(10);
	}
}
