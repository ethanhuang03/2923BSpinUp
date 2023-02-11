#include "autons.hpp"

void winch_checker(){
	while (limit_switch.get_value() != 1) {
		winchGroup.moveVoltage(12000);
		pros::delay(20);
	}
	winchGroup.moveVoltage(1200);
}

void shoot(int angle) {
	rotation_sensor.reset();
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
	pros::Task winch_task2(winch_checker);
	intake_roller.moveVoltage(-12000);
	moveTime(std::make_pair(100, 100), 300_ms);
	intake_roller.moveVoltage(0);
	moveDistance(-0.6_ft, 1_s);
	turnToAngle(-129_deg);
	intake_roller.moveVoltage(-12000);
	moveDistance(2.5_ft, 1_s);
	intake_roller.moveVoltage(12000);
	pros::delay(300);
	moveDistance(1_ft, 1_s);
	pros::delay(300);
	moveDistance(1.5_ft, 1_s);
	turnToAngle(-217_deg);
	moveDistance(0.8_ft, 1_s);
	shoot(0);
}

void Right(){
	pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(12000);
	pros::delay(1300);
	moveDistance(2_ft, 1_s);
	pros::delay(300);
	moveDistance(1_ft, 1_s);
	turnToAngle(25_deg);
	pros::delay(300);
	intake_roller.moveVoltage(0);
	/*
	PTO1.toggle();
	pros::delay(200);
	PTO1.toggle();
	*/
	shoot(0);
	pros::Task winch_task1(winch_checker);
	
	intake_roller.moveVoltage(12000);
	moveDistance(-0.5_ft, 1_s);
	turnToAngle(-45_deg);
	// start intaking 2nd and 3rd disc
	moveDistance(2_ft, 1_s);
	moveDistance(1_ft, 1_s);
	pros::delay(300);
	// turn towards goal + shoot
	turnToAngle(43_deg);
	shoot(0);
	// finish with winch at the top and at the center
	turnToAngle(140_deg);
	intake_roller.moveVoltage(0);
	moveTime(std::make_pair(100, 100), 2_s);
	intake_roller.moveVoltage(-12000);
	pros::delay(300);
	intake_roller.moveVoltage(0);
}

void Skills() {
	/*
	// starting on left rollers
	// spin roller
	pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(12000);
	pros::delay(200);
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
	pros::delay(200);
	moveDistance(-1_ft, 2_s);
	// turn around and get edge disc
	turnToAngle(45_deg);
	moveDistance(1_ft, 2_s);
	*/
}

void AWP(){}