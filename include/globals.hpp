#pragma once
#include "main.h"

// Controller
extern Controller master;
extern Controller partner;

// Motors
extern Motor leftBack; 
extern Motor leftFront; 
extern Motor rightBack; 
extern Motor rightFront; 

extern Motor left_winch;
extern Motor right_winch;
extern Motor middle_winch;

extern Motor intake_roller;

extern MotorGroup leftDrive;
extern MotorGroup rightDrive;

extern MotorGroup winchGroup;

// Sensors
extern IMU imu;
extern RotationSensor rotation_sensor;
extern pros::ADIDigitalIn limit_switch;
extern pros::Vision vision_sensor;

// Pneumatics
extern Pneumatics PTO1;
extern Pneumatics expansion;

// Controllers
extern std::shared_ptr<ChassisController> chassis;


// PID Controllers 
extern std::shared_ptr<IterativePosPIDController> turnPID;
extern std::shared_ptr<IterativePosPIDController> movePID;
extern std::shared_ptr<IterativePosPIDController> headingPID;
extern std::shared_ptr<IterativePosPIDController> visionPID;