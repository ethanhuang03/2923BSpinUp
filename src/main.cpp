#include "main.h"

// Controller
Controller master(ControllerId::master); 
Controller partner(ControllerId::partner);

// Motors
Motor leftBack(1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor leftFront(2, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor rightBack(3, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
Motor rightFront(4, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees); 
MotorGroup leftDrive({leftFront, leftBack});
MotorGroup rightDrive({rightFront, rightBack});

// Sensors
IMU imu(5);

// Contraints
ProfileConstraint constraint({0.275666666666_mps, 9_mps2, 19_mps2, 18_mps3});
FFVelocityController leftLinear(0.187, 0.04, 0.025, 4.35, 0.1);
FFVelocityController rightLinear(0.1915, 0.043, 0.02, 4, 0.1);
FFVelocityController leftTrajectory(0.187, 0.04, 0.025, 2.5, 0);
FFVelocityController rightTrajectory(0.187, 0.043, 0.02, 2.5, 0);

// Controllers
std::shared_ptr<ChassisController> chassis;
std::shared_ptr<AsyncMotionProfiler> profiler;
std::shared_ptr<AsyncMotionProfiler> profiler2;

// PID Controllers 
std::shared_ptr<IterativePosPIDController> turnPID = std::make_shared<IterativePosPIDController>(0.06, 0.005, 0.00115, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 200_ms));
std::shared_ptr<IterativePosPIDController> movePID = std::make_shared<IterativePosPIDController>(0.1, 0.0, 0.002, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms));
std::shared_ptr<IterativePosPIDController> headingPID = std::make_shared<IterativePosPIDController>(0.118, 0, 0, 0, TimeUtilFactory::createDefault());

// Constants
const double DEADBAND = 0.0500;

// Functions
void moveTime(std::pair<double, double> speed, QTime time) {
    (chassis->getModel())->tank(speed.first, speed.second);
    pros::delay(time.convert(millisecond));
    (chassis->getModel())->tank(0, 0);
}

void moveTimeHeadingCorrect(double speed, QTime time) {
    headingPID->reset();
    headingPID->setTarget(imu.get());
    auto timer = TimeUtilFactory().createDefault().getTimer();
    timer->placeMark();  

    do {
        (chassis->getModel())->arcade(speed, headingPID->step(imu.get()));
        pros::delay(10);
    } while(timer->getDtFromMark() < time);
    (chassis->getModel())->stop();
}

void moveDistance(QLength target, QTime time) {
    movePID->reset(); headingPID->reset();
    movePID->setTarget(0); headingPID->setTarget(imu.get());
    (chassis->getModel())->resetSensors();
    auto timer = TimeUtilFactory().createDefault().getTimer();
    timer->placeMark();  

	do {
        double dist = Math::tickToFt(((chassis->getModel())->getSensorVals()[0] + (chassis->getModel())->getSensorVals()[1]) / 2, chassis->getChassisScales(), chassis->getGearsetRatioPair()) * 12;
        double error = target.convert(inch) - dist;
        (chassis->getModel())->arcade(movePID->step(-error), headingPID->step(imu.get()));
		pros::delay(10);
	} while(!movePID->isSettled() || timer->getDtFromMark() < time);

	(chassis->getModel())->stop();
}

void turnToAngle(QAngle targetAngle){
	turnPID->reset();
    turnPID->setTarget(0);
    turnPID->setIntegratorReset(true);
    turnPID->setIntegralLimits(0.4 / 0.015, -0.4 / 0.015);
    turnPID->setErrorSumLimits(15, 0);

	do{
        (chassis->getModel())->arcade(0, turnPID->step(-Math::rescale180(targetAngle.convert(degree)-imu.get())));
        pros::delay(10);
    }while (!turnPID->isSettled());

    (chassis->getModel())->stop();
}


// Competition
void initialize() {
	// Controllers
	chassis = ChassisControllerBuilder()
		.withMotors(leftDrive, rightDrive)
		.withDimensions({AbstractMotor::gearset::green, 1.0/1.0}, {{3.25_in, 14.75_in}, imev5GreenTPR})
		.build();

	profiler = AsyncMotionProfilerBuilder()
		.withOutput(chassis)
		.withProfiler(std::make_unique<SCurveMotionProfile>(constraint))
		.withLinearController(leftLinear, rightLinear)
		.withTrajectoryController(leftTrajectory, rightTrajectory)
		.build();

	profiler2 = AsyncMotionProfilerBuilder()
		.withOutput(chassis)
		.withProfiler(std::make_unique<SCurveMotionProfile>(constraint))
		.build();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	turnToAngle(90_deg);
	while (true) {
		chassis->getModel()->curvature(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightX), DEADBAND);
		pros::delay(20);
	}
}
