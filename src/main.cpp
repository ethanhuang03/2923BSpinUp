#include "main.h"


okapi::Controller master(okapi::ControllerId::master);
okapi::Controller partner(okapi::ControllerId::partner);


void initialize() {}


void disabled() {}


void competition_initialize() {}


void autonomous() {}


void opcontrol() {
	while(true) {
		chassis::tank(master.getAnalog(okapi::ControllerAnalog::leftY)*100, master.getAnalog(okapi::ControllerAnalog::rightY)*100);
		pros::delay(10);
	}
}
