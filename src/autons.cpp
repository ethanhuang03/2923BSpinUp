#include "autons.hpp"

void winch_checker(){
	while (limit_switch.get_value() != 1) {
		winchGroup.moveVoltage(12000);
		pros::delay(20);
	}
	winchGroup.moveVoltage(3000);
}

void Left(){}

void Right(){
	pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(12000);
	pros::delay(1600);
	moveDistance(2.5_ft, 1_s);
	turnToAngle(25_deg);

	intake_roller.moveVoltage(0);
	moveDistance(0.5_ft, 1_s);
	PTO1.toggle();
	pros::delay(200);
	PTO1.toggle();
	pros::Task winch_task1(winch_checker);
	pros::delay(1600);
	intake_roller.moveVoltage(12000);


}

void Skills(){}

void AWP(){}

void LeftAWP(){}

void RightAWP(){}