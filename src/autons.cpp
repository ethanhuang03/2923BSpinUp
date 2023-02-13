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

void triple_stack() {
	intake_roller.moveVoltage(-12000);
	moveDistance(0.5_ft, 1_s, 50);
	intake_roller.moveVoltage(12000);
	pros::delay(300);
	moveDistance(2_ft, 4_s, 15);
	
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

	// First Roller
	pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(-12000);
	moveTime(std::make_pair(100, 100), 300_ms);
	pros::delay(100);
	intake_roller.moveVoltage(0);
	moveDistance(-0.6_ft, 1_s);
	intake_roller.moveVoltage(12000);
	turnToAngle(125_deg, 1_s);
	moveDistance(2_ft, 1_s);
	turnToAngle(90_deg, 1_s);
	
	// Intake 1 Disk and Second Roller
	intake_roller.moveVoltage(-12000);
	moveTime(std::make_pair(100, 100), 0.5_s);
	pros::delay(400);
	intake_roller.moveVoltage(0);
	moveDistance(-0.5_ft, 1_s);


	// Move To Align and Shoot
	turnToAngle(190_deg, 1_s);
	moveDistance(1.6_ft, 1_s);
	turnToAngle(175_deg, 1_s);
	//TUNE
	shoot(0);
	pros::delay(1000);
	
	// Get Single 3 Disks and Shoot
	turnToAngle(225_deg, 1_s);
	intake_roller.moveVoltage(12000);
	moveDistance(4.5_ft, 3_s, 50);
	pros::delay(300);
	turnToAngle(135_deg, 1_s);
	//TUNE
	shoot(0);
	pros::delay(1200);

	// Triple Stack
	turnToAngle(225_deg, 1_s);
	moveDistance(2.4_ft, 1_s);
	triple_stack();


	/*
	turnToAngle(217_deg, 1_s);
	intake_roller.moveVoltage(-12000);
	moveDistance(2_ft, 1_s);
	moveDistance(0.5_ft, 1_s);
	intake_roller.moveVoltage(12000);
	turnToAngle(220_deg, 1_s);
	pros::delay(500);
	moveDistance(1_ft, 1_s);
	pros::delay(300);
	moveDistance(1_ft, 1_s);
	pros::delay(300);

	turnToAngle(87_deg, 1_s);

	//TUNE
	shoot(0);
	pros::delay(1000);
	
	moveDistance(-0.6_ft, 1_s);
	turnToAngle(183_deg, 1_s);
	moveDistance(0.7_ft, 1_s);
	intake_roller.moveVoltage(-12000);
	moveTimeHeadingCorrect(100, 300_ms);
	pros::delay(100);
	intake_roller.moveVoltage(0);
	moveDistance(-0.6_ft, 1_s);
	*/
}

void AWP(){
	
}