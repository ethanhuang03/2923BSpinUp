#include "autons.hpp"

void winch_checker(){
	while (limit_switch.get_value() != 1) {
		winchGroup.moveVoltage(12000);
		pros::delay(20);
	}
	winchGroup.moveVoltage(1200);
}

void shoot(double angle) {
	double current_angle = rotation_sensor.get();
	while (rotation_sensor.get() < current_angle+angle) {
		winchGroup.moveVoltage(-12000);
		pros::delay(20);
	}
	winchGroup.moveVoltage(1200);
	pros::delay(100);
	PTO1.toggle();
	pros::delay(200);
	PTO1.toggle();
	pros::Task winch_task(winch_checker);
}

void triple_stack() {
	intake_roller.moveVoltage(-12000);
	moveDistance(0.9_ft, 1_s, 50);
	intake_roller.moveVoltage(12000);
	pros::delay(300);
	moveDistance(1.9_ft, 5_s, 40);
}

void Left(){
	//pros::Task winch_task2(winch_checker);
	intake_roller.moveVoltage(-12000);
	moveTime(std::make_pair(100, 100), 300_ms);
	intake_roller.moveVoltage(0);
	moveDistance(-0.6_ft, 1_s);
	/*
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
	*/
}

void Right(){
	pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(12000);
	pros::delay(2000);
	moveDistance(1.8_ft, 1_s, 80);
	pros::delay(300);
	moveDistance(0.8_ft, 1_s);
	turnToAngle(22_deg, 1_s);
	pros::delay(300);
	intake_roller.moveVoltage(0);
	shoot(10);
	pros::Task winch_task1(winch_checker);

	intake_roller.moveVoltage(-12000);
	turnToAngle(150_deg, 1_s);
	moveDistance(3.5_ft, 1_s);
	turnToAngle(179_deg, 1_s);
	moveTime(std::make_pair(100, 100), 0.3_s);
	pros::delay(400);
	intake_roller.moveVoltage(0);
	moveDistance(-0.5_ft, 1_s);
	
	
	/*
	intake_roller.moveVoltage(12000);
	moveDistance(-0.5_ft, 1_s);
	turnToAngle(-45_deg, 1_s);
	pros::delay(1000);
	// start intaking 2nd and 3rd disc
	moveDistance(2_ft, 1.5_s, 80);
	moveDistance(2_ft, 1_s, 80);
	pros::delay(300);
	// turn towards goal + shoot
	turnToAngle(45_deg, 1_s);
	moveDistance(0.5_ft, 1_s, 100);
	shoot(10);
	// finish with winch at the top and at the center
	turnToAngle(137_deg);
	intake_roller.moveVoltage(0);
	moveTime(std::make_pair(100, 100), 2_s);
	intake_roller.moveVoltage(-12000);
	pros::delay(300);
	intake_roller.moveVoltage(0);
	*/
}

void Skills() {
	// First Stage
	//pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(12000);
	pros::delay(1600);
	turnToAngle(34_deg, 1_s);
	moveDistance(2_ft, 1_s);
	moveDistance(0.5_ft, 1_s);
	turnToAngle(90_deg, 1_s);
	moveDistance(1.2_ft, 1_s);
	turnToAngle(90_deg, 1_s);
	shoot(55);
	pros::delay(1600);

	// Secnod Stage
	turnToAngle(135_deg, 1_s);
	moveDistance(2_ft, 1_s);
	turnToAngle(135_deg, 1_s);
	moveDistance(1.5_ft, 1_s);
	turnToAngle(135_deg, 1_s);
	moveDistance(1.8_ft, 1_s);
	pros::delay(500);
	turnToAngle(42_deg, 1_s);
	shoot(140);
	pros::delay(1600);

	// Third Stage
	turnToAngle(-132_deg, 1_s);
	moveDistance(2_ft, 1_s);
	turnToAngle(-135_deg, 1_s);
	moveDistance(1.4_ft, 1_s);
	pros::delay(300);
	turnToAngle(137_deg, 1_s);
	moveDistance(1.5_ft, 1_s);
	turnToAngle(137_deg, 1_s);
	moveDistance(2_ft, 1_s);
	pros::delay(500);
	turnToAngle(-105.5_deg, 1_s);
	shoot(0);
	pros::delay(1600);

	// Fourth Stage
	turnToAngle(135_deg, 1_s);
	moveDistance(1.6_ft, 1_s);
	turnToAngle(45_deg, 1_s);
	intake_roller.moveVoltage(-12000);
	moveDistance(1.2_ft, 1_s);
	intake_roller.moveVoltage(12000);
	pros::delay(300);
	moveDistance(2_ft, 5_s, 20);
	turnToAngle(1_deg, 1_s);
	shoot(0);
	pros::delay(1600);

	// Fifth Stage
	turnToAngle(-41_deg, 1_s);
	intake_roller.moveVoltage(-12000);
	moveDistance(1.5_ft, 1_s);	
	intake_roller.moveVoltage(12000);
	pros::delay(300);
	moveDistance(2_ft, 5_s, 20);
	pros::delay(300);
	turnToAngle(38_deg, 1_s);
	shoot(0);
	pros::delay(1600);

	// Sixth Stage
	turnToAngle(-136_deg, 1_s);
	moveDistance(12_ft, 1_s);
	pros::delay(300);
	turnToAngle(-45_deg, 1_s);
	

	/*
	// First Roller
	pros::Task winch_task(winch_checker);
	intake_roller.moveVoltage(-12000);
	moveTime(std::make_pair(100, 100), 300_ms);
	pros::delay(100);
	intake_roller.moveVoltage(0);
	moveDistance(-0.6_ft, 1_s);
	intake_roller.moveVoltage(12000);
	turnToAngle(110_deg, 1_s);
	pros::delay(800);
	moveDistance(1.6_ft, 2_s);
	pros::delay(300);
	turnToAngle(90_deg, 1_s);
	
	// Intake 1 Disk and Second Roller
	moveDistance(1.3_ft, 1_s);
	intake_roller.moveVoltage(-12000);
	moveTime(std::make_pair(100, 100), 0.3_s);
	pros::delay(500);
	intake_roller.moveVoltage(0);
	moveDistance(-0.5_ft, 1_s);
	intake_roller.moveVoltage(12000);

	// Move To Align and Shoot
	turnToAngle(190_deg, 1_s);
	moveDistance(2.2_ft, 1_s);
	turnToAngle(171_deg, 1_s);
	//TUNE
	shoot(55);
	pros::delay(1600);

	// Get Single 3 Disks and Shoot
	turnToAngle(215_deg, 1_s);
	moveDistance(1.5_ft, 1.5_s);
	turnToAngle(225_deg, 1_s);
	moveDistance(1.5_ft, 1.5_s);
	turnToAngle(225_deg, 1_s);
	moveDistance(1.5_ft, 1.5_s);
	pros::delay(500);
	turnToAngle(129_deg, 1_s);
	//TUNE
	shoot(120);
	pros::delay(1000);

	
	// Triple Stack
	intake_roller.moveVoltage(-12000);
	turnToAngle(-90_deg, 1.5_s);
	moveDistance(1_ft, 1_s);
	turnToAngle(185_deg, 1_s);
	intake_roller.moveVoltage(12000);
	moveDistance(1.5_ft, 1_s);
	triple_stack();
	turnToAngle(178_deg, 1_s);
	moveDistance(0.5_ft, 1_s);
	turnToAngle(88_deg, 1_s);
	//TUNE
	shoot(65);
	pros::delay(1000);
	turnToAngle(-90_deg, 2_s);

	// Another Disk + oller
	intake_roller.moveVoltage(12000);
	moveDistance(1.5_ft, 1_s);
	turnToAngle(-70_deg, 1_s);
	moveDistance(2_ft, 2_s);
	pros::delay(300);
	turnToAngle(-90_deg, 1_s);
	pros::delay(200);
	intake_roller.moveVoltage(-12000);
	moveTime(std::make_pair(100, 100), 0.5_s);
	pros::delay(400);
	intake_roller.moveVoltage(0);
	moveDistance(-0.5_ft, 1_s);

	// Shoot 
	turnToAngle(2_deg, 1_s);
	//TUNE
	shoot(0);
	pros::delay(500);

	
	
	// 3 DIsks 
	moveDistance(2.4_ft, 1_s);
	turnToAngle(120_deg, 1_s); // 128
	intake_roller.moveVoltage(12000);
	moveDistance(1.5_ft, 1_s);
	triple_stack();
	moveDistance(-0.5_ft, 1_s);
	
	// Rollers
	turnToAngle(181_deg, 1_s);
	intake_roller.moveVoltage(-12000);
	moveDistance(1.8_ft, 1_s);
	moveTime(std::make_pair(100, 100), 0.8_s);
	pros::delay(400);
	intake_roller.moveVoltage(0);
	moveDistance(-0.5_ft, 1_s);
	
	intake_roller.moveVoltage(12000);
	turnToAngle(0_deg, 1_s);
	moveDistance(2_ft, 1_s);
	turnToAngle(-15_deg, 1_s);
	//TUNE
	shoot(50);
	
	// go back and expand

	turnToAngle(-140_deg, 1_s);
	moveDistance(3_ft, 2_s);
	turnToAngle(-135_deg, 1_s);
	// expansion.toggle();
	*/
}

void AWP(){
	
}