#include "main.h"
#include "ARMS/config.h"


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
	bool master_expansion = false;
	bool partner_expansion = false;
	bool aimbot = false;
	bool overide = false;

	while(true) {
		arms::chassis::tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		// Manual Overide //
		if( master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && 
			master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && 
			master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && 
			master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) ) {
			overide = true;
		}
		else {
			overide = false;
		}

		// Expansion Logic //
		if ( !overide ) {
			if( master.get_digital(pros::E_CONTROLLER_DIGITAL_X) ) {
				master_expansion = !master_expansion;
			}
			if( partner.get_digital(pros::E_CONTROLLER_DIGITAL_X) ) {
				partner_expansion = !partner_expansion;
			}
			if( master_expansion && partner_expansion ) {
				piston(expansion, false, true);
			}
		}
		else {
			piston(expansion, false, true);
		}



		pros::delay(10);
	}
}


/*
CONTROLLER DISPLAY
Aimbot: True/False
Expansion State: True/False, True/False
Overide Features: True/False
*/
