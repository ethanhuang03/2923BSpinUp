#include "main.h"

// Controller
Controller master(ControllerId::master); 

// Motors
Motor leftBack(1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor leftFront(2, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor rightBack(3, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor rightFront(4, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 

Motor leftPTO(5, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor rightPTO(6, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor winch(7, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor intake_roller(8, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

MotorGroup leftDrive({leftFront, leftBack});
MotorGroup rightDrive({rightFront, rightBack});

MotorGroup leftPTOGroup({leftPTO, leftFront, leftBack});
MotorGroup rightPTOGroup({rightPTO, rightFront, rightBack});

MotorGroup winchPTO({winch, leftPTO, rightPTO});

// Sensors
IMU imu(9);

// Pneumatics
Pneumatics PTO1(1, true);

// Contraints
ProfileConstraint constraint({0.275666666666_mps, 9_mps2, 19_mps2, 18_mps3});
FFVelocityController leftLinear(0.187, 0.04, 0.025, 4.35, 0.1);
FFVelocityController rightLinear(0.1915, 0.043, 0.02, 4, 0.1);
FFVelocityController leftTrajectory(0.187, 0.04, 0.025, 2.5, 0);
FFVelocityController rightTrajectory(0.187, 0.043, 0.02, 2.5, 0);

// Controllers
std::shared_ptr<ChassisController> drive;
std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
		.withMotors(leftDrive, rightDrive)
		.withDimensions({AbstractMotor::gearset::green, 0.6}, {{3.25_in, 14.75_in}, imev5GreenTPR})
		.build();
std::shared_ptr<ChassisController> chassisPTO = ChassisControllerBuilder()
		.withMotors(leftPTOGroup, rightPTOGroup)
		.withDimensions({AbstractMotor::gearset::green, 0.6}, {{3.25_in, 14.75_in}, imev5GreenTPR})
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
std::shared_ptr<IterativePosPIDController> turnPID = std::make_shared<IterativePosPIDController>(0.06, 0.005, 0.00115, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 200_ms));
std::shared_ptr<IterativePosPIDController> movePID = std::make_shared<IterativePosPIDController>(0.1, 0.0, 0.002, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms));
std::shared_ptr<IterativePosPIDController> headingPID = std::make_shared<IterativePosPIDController>(0.118, 0, 0, 0, TimeUtilFactory::createDefault());


