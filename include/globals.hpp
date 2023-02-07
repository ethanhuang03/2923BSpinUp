#pragma once
#include "main.h"

// Controller
extern Controller master; 

// Motors
extern Motor leftBack; 
extern Motor leftFront; 
extern Motor rightBack; 
extern Motor rightFront; 

extern Motor leftPTO;
extern Motor rightPTO;
extern Motor winch;

extern Motor intake_roller;

extern MotorGroup leftDrive;
extern MotorGroup rightDrive;

extern MotorGroup leftPTOGroup;
extern MotorGroup rightPTOGroup;

extern MotorGroup winchPTO;

// Sensors
extern IMU imu;
extern RotationSensor rotation_sensor;
extern pros::ADIDigitalIn limit_switch;

// Pneumatics
extern Pneumatics PTO1;
extern Pneumatics expansion;

// Contraints
extern ProfileConstraint constraint;
extern FFVelocityController leftLinear;
extern FFVelocityController rightLinear;
extern FFVelocityController leftTrajectory;
extern FFVelocityController rightTrajectory;

// Controllers
extern std::shared_ptr<ChassisController> drive;
extern std::shared_ptr<ChassisController> chassis;
extern std::shared_ptr<ChassisController> chassisPTO;

extern std::shared_ptr<AsyncMotionProfiler> profiler;
extern std::shared_ptr<AsyncMotionProfiler> profiler_blind;

// PID Controllers 
extern std::shared_ptr<IterativePosPIDController> turnPID;
extern std::shared_ptr<IterativePosPIDController> movePID;
extern std::shared_ptr<IterativePosPIDController> headingPID;
