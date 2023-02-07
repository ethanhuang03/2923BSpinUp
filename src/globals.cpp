#include "main.h"

// Controller
Controller master(ControllerId::master); 

// Motors
Motor leftBack(11, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor leftFront(12, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor rightBack(13, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor rightFront(14, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 

Motor leftPTO(15, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor rightPTO(16, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor winch(17, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor intake_roller(18, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

MotorGroup leftDrive({leftFront, leftBack});
MotorGroup rightDrive({rightFront, rightBack});

MotorGroup leftPTOGroup({leftPTO, leftFront, leftBack});
MotorGroup rightPTOGroup({rightPTO, rightFront, rightBack});

MotorGroup winchPTO({winch, leftPTO, rightPTO});

// Sensors
IMU imu(19);
RotationSensor rotation_sensor(20);
pros::ADIDigitalIn limit_switch(3);

// Pneumatics
Pneumatics PTO1(1, true);
Pneumatics expansion(2, false);

// Contraints
ProfileConstraint constraint({4.72693391_ftps, 10.142_mps2, 10.142_mps2, 20.284_mps3});
FFVelocityController leftLinear(0.187, 0.04, 0.025, 4.35, 0.1);
FFVelocityController rightLinear(0.1915, 0.043, 0.02, 4, 0.1);
FFVelocityController leftTrajectory(0.187, 0.04, 0.025, 2.5, 0);
FFVelocityController rightTrajectory(0.187, 0.043, 0.02, 2.5, 0);

// Controllers
std::shared_ptr<ChassisController> drive;
std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
		.withMotors(leftDrive, rightDrive)
		.withDimensions({AbstractMotor::gearset::green, 0.6}, {{3.25_in, 15.0625_in}, imev5GreenTPR})
		.build();
std::shared_ptr<ChassisController> chassisPTO = ChassisControllerBuilder()
		.withMotors(leftPTOGroup, rightPTOGroup)
		.withDimensions({AbstractMotor::gearset::green, 0.6}, {{3.25_in, 15.0625_in}, imev5GreenTPR})
		.build();

std::shared_ptr<AsyncMotionProfiler> profiler = AsyncMotionProfilerBuilder()
		.withOutput(chassis)
		.withProfiler(std::make_unique<SCurveMotionProfile>(constraint))
		.withLinearController(leftLinear, rightLinear)
		.withTrajectoryController(leftTrajectory, rightTrajectory)
		.build();
std::shared_ptr<AsyncMotionProfiler> profiler_blind = AsyncMotionProfilerBuilder()
		.withOutput(chassis)
		.withProfiler(std::make_unique<SCurveMotionProfile>(constraint))
		.build();

// PID Controllers 
std::shared_ptr<IterativePosPIDController> turnPID = std::make_shared<IterativePosPIDController>(0.064011, 0.001, 0.003096, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 200_ms));
std::shared_ptr<IterativePosPIDController> movePID = std::make_shared<IterativePosPIDController>(0.12, 0.0, 0.002, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms));
std::shared_ptr<IterativePosPIDController> headingPID = std::make_shared<IterativePosPIDController>(0.2, 0, 0.0022453, 0, TimeUtilFactory::createDefault());


