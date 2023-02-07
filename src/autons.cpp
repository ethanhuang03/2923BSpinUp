#include "autons.hpp"

void winch_checker(){
	while (limit_switch.get_value() != 1) {
		winchGroup.moveVoltage(12000);
		pros::delay(20);
	}
	winchGroup.moveVoltage(3000);
}

void shoot(int angle) {
	winchGroup.moveVoltage(-12000);
	while (rotation_sensor.get() < angle) {
		pros::delay(20);
	}
	winchGroup.moveVoltage(0);
	pros::delay(100);
	PTO1.toggle();
	pros::delay(200);
	PTO1.toggle();
	pros::Task winch_task(winch_checker);
}

void Left(){
	// do roller
	pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(12000);
	pros::delay(200);
	// move away and get the 1 disc in the middle
	moveDistance(1_ft, 1_s);
	turnToAngle(-45_deg);
	moveDistance(1.5_ft, 1_s);
	pros::delay(200);
	// move back, turn and shoot
	intake_roller.moveVoltage(0);
	moveDistance(-1_ft, 1_s);
	turnToAngle(-5_deg);
	moveDistance(1_ft, 1_s);
	PTO1.toggle();
	pros::delay(200);
	PTO1.toggle();
	pros::Task winch_task(winch_checker);
	// turn towards 3 stack and prepare to intake
	turnToAngle(70_deg);
	moveDistance(2_ft, 1_s);
	// wait until fully winched
	pros::delay(1000);
	// move forward and outtake to push the top discs
	intake_roller.moveVoltage(-12000);
	moveDistance(1_ft, 1_s);
	// now intake and move forward to intake all
	intake_roller.moveVoltage(12000);
	moveDistance(1_ft, 1_s);
	// small delay to decrease speed (maybe not necessary)
	pros::delay(200);
	moveDistance(1_ft, 1_s);
	// turn and drive towards goal
	turnToAngle(-40_deg);
	moveDistance(1.5_ft, 1_s);
	// potentially need to reangle
	
	// shoot
	PTO1.toggle();
	pros::delay(200);
	PTO1.toggle();
	// finish with robot approx near the center with winch up
}

void Right(){
	pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(12000);
	pros::delay(1600);
	moveDistance(3_ft, 1_s);
	turnToAngle(25_deg);
	intake_roller.moveVoltage(0);
	moveDistance(0.5_ft, 1_s);
	PTO1.toggle();
	pros::delay(200);
	PTO1.toggle();
	pros::Task winch_task1(winch_checker);
	pros::delay(1600);
	// grab the next disc and come back
	intake_roller.moveVoltage(12000);
	turnToAngle(45_deg);
	moveDistance(0.75_ft, 1_s);
	pros::delay(200);
	moveDistance(-1.5_ft, 1_s);
	turnToAngle(-45_deg);
	// start intaking 2nd and 3rd disc
	moveDistance(3_ft, 1_s);
	// turn towards goal, move forward + shoot
	turnToAngle(45_deg);
	moveDistance(1_ft, 1_s);
	PTO1.toggle();
	pros::delay(200);
	PTO1.toggle();
	// finish with winch at the top and at the center
}

void Skills() {
	// starting on left rollers
	// spin roller
	pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(12000);
	pross:delay(200);
	// move away and turn towards disc
	moveDistance(1_ft, 1_s);
	turnToAngle(-45_deg);
	// wait until winched
	pros::delay(1000);
	moveDistance(2_ft, 1_s);
	// turn towards roller and spin it
	turnToAngle(-90_deg);
	moveDistance(1.5_ft, 1_s);
	pros::delay(200);
	// move back, turn and shoot
	moveDistance(-1_ft, 1_s);
	turnToAngle(0_deg);
	// maybe need to move forward a little to be in line with discs
	shoot(100);
	// turn and intake 3 in a row
	turnToAngle(45_deg);
	moveDistance(5_ft, 2_s);
	// turn towards goal and shoot
	turnToAngle(-55_deg);
	shoot(300);
	// turn and go towards 3 stack
	turnToAngle(45_deg);
	moveDistance(1.5_ft, 2_s);
	// outtake to push the top away
	intake_roller.moveVoltage(-12000);
	moveDistance(0.5_ft, 2_s);
	// move forward and intake
	intake_roller.moveVoltage(12000);
	moveDistance(1.5_ft, 2_s);
	// turn towards goal and shoot
	turnToAngle(-45_deg);
	shoot(200);
	// turn and move towards roller
	turnToAngle(45_deg);
	moveDistance(1_ft, 2_s);
	turnToAngle(0_deg);
	// turn rollers
	moveDistance(1_ft, 2_s);
	pross::delay(200);
	moveDistance(-1_ft, 2_s);
}

void AWP(){}

void LeftAWP(){}

void RightAWP(){}