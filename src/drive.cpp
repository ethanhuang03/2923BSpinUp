#include "drive.hpp"

// Movement Functions
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