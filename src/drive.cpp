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

void moveDistance(QLength target, QTime time, double max_speed) {
    movePID->reset(); headingPID->reset();
    movePID->setTarget(0); headingPID->setTarget(imu.get());
    (chassis->getModel())->resetSensors();
    auto timer = TimeUtilFactory().createDefault().getTimer();
    timer->placeMark();  

	do {
        double dist = Math::tickToFt(((chassis->getModel())->getSensorVals()[0] + (chassis->getModel())->getSensorVals()[1]) / 2, chassis->getChassisScales(), chassis->getGearsetRatioPair()) * 12;
        double error = target.convert(inch) - dist;
		if (movePID->step(-error) > max_speed/100) {
			(chassis->getModel())->arcade(max_speed/100, headingPID->step(imu.get()));
		}
		else{
			(chassis->getModel())->arcade(movePID->step(-error), headingPID->step(imu.get()));
		}
		
		pros::delay(10);
	} while(!movePID->isSettled() && timer->getDtFromMark() < time);

	(chassis->getModel())->stop();
}

void turnToAngle(QAngle targetAngle, QTime time){
	turnPID->reset();
    turnPID->setTarget(0);
    turnPID->setIntegratorReset(true);
    turnPID->setIntegralLimits(0.4 / 0.015, -0.4 / 0.015);
    turnPID->setErrorSumLimits(15, 0);
	auto timer = TimeUtilFactory().createDefault().getTimer();
	timer->placeMark();

	do{
        (chassis->getModel())->arcade(0, turnPID->step(-Math::rescale180(targetAngle.convert(degree)-imu.get())));
        pros::delay(10);
    }while (!turnPID->isSettled() && timer->getDtFromMark() < time);

    (chassis->getModel())->stop();
}

void turnToGoal() {
	QTime max_time = 1_s;
	auto timer = TimeUtilFactory().createDefault().getTimer();
	timer->placeMark();

    do {
        pros::vision_object_s_t rtn = vision_sensor.get_by_size(0);
        (chassis->getModel())->arcade(0, visionPID->step(-rtn.x_middle_coord));
		pros::delay(10);
	} while(!visionPID->isSettled() && timer->getDtFromMark() < max_time);
	(chassis->getModel())->stop();
}