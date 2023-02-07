#include "main.h"

// Controller
Controller master(ControllerId::master); 

// Motors
Motor leftBack(11, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor leftFront(12, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor rightBack(13, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor rightFront(14, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 

Motor left_winch(15, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor right_winch(16, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor middle_winch(17, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor intake_roller(18, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

MotorGroup leftDrive({leftFront, leftBack});
MotorGroup rightDrive({rightFront, rightBack});

MotorGroup winchGroup({left_winch, right_winch, middle_winch});

// Sensors
IMU imu(19);
RotationSensor rotation_sensor(20);
pros::ADIDigitalIn limit_switch(3);

// Pneumatics
Pneumatics PTO1(1, true);
Pneumatics expansion(2, false);

// Controllers
std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
		.withMotors(leftDrive, rightDrive)
		.withDimensions({AbstractMotor::gearset::green, 0.6}, {{3.25_in, 15.0625_in}, imev5GreenTPR})
		.build();

// PID Controllers 
std::shared_ptr<IterativePosPIDController> turnPID = std::make_shared<IterativePosPIDController>(0.064011, 0.001, 0.003096, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 200_ms));
std::shared_ptr<IterativePosPIDController> movePID = std::make_shared<IterativePosPIDController>(0.12, 0.0, 0.002, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms));
std::shared_ptr<IterativePosPIDController> headingPID = std::make_shared<IterativePosPIDController>(0.2, 0, 0.0022453, 0, TimeUtilFactory::createDefault());


