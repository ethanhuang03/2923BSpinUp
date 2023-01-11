#include "motion_profiling/api.h"
#include <vector>


double inps2rpm(double inps) { // wheel radius in inches
    return (inps * 60 / (M_PI * WHEELDIAMETER))*GEARRATIO;
}


double mps2rpm(double mps) { // wheel radius in meters
	return (mps * 60 / (M_PI * WHEELDIAMETER))*GEARRATIO;
}

double m2in(double m) {
    return m * 39.37;
}


double in2m(double in) {
    return in / 39.37;
}


double deg2rad(double deg) {
	return deg * M_PI / 180;
}


double rad2deg(double rad) {
	return rad * 180 / M_PI;
}

// Constrains in SI Units
void PathFollower::createConstraints(std::string constraint_name, double max_vel, double max_acc, double max_jerk) { // m/s, m/s^2, m/s^3
    constraints[constraint_name] = {max_vel, max_acc, max_jerk};
}

// Generate Path (waypoint in in, angles in degrees)
void PathFollower::createPath(std::string constraint_name, std::string path_name, std::vector<std::vector<double>> waypoints) { // in, in, deg //heading might be different
    std::vector<double> constraint = constraints[constraint_name];
    squiggles::Constraints squiggles_constraints = squiggles::Constraints(constraint[0], constraint[1], constraint[2]);
    squiggles::SplineGenerator generator = squiggles::SplineGenerator(squiggles_constraints, std::make_shared<squiggles::TankModel>(robot_width, squiggles_constraints));
    std::vector<squiggles::Pose> points;
    for (int i = 0; i < waypoints.size(); i++) {
        points.push_back(squiggles::Pose(in2m(waypoints[i][0]), in2m(waypoints[i][1]), deg2rad(waypoints[i][2]))); // convert in to m, deg to rad
    }
    paths[path_name] = generator.generate(points); // returns meters, meters, radians
}

void PathFollower::followPath(std::string path_name) {
    std::vector<squiggles::ProfilePoint> path = paths[path_name];
    std::size_t pathSize = path.size();
    for (std::size_t i = 0; i < pathSize; i++) {  // used to be ++i
        auto leftRPM = inps2rpm(m2in(path[i].wheel_velocities[0]));
        auto rightRPM = inps2rpm(m2in(path[i].wheel_velocities[1]));

		arms::chassis::leftMotors->move_velocity(leftRPM);
		arms::chassis::rightMotors->move_velocity(rightRPM);
        
        if (i < pathSize - 1) {
            double delay = path[i+1].time - path[i].time;
            pros::delay(delay*1000);
        }
    }
}

void PathFollower::followPathRamsete(std::string path_name) {
    std::vector<squiggles::ProfilePoint> path = paths[path_name];

    RamseteController controller;
    controller.setGains(0.7, 0.7);

    std::size_t pathSize = path.size();
    for (std::size_t i = 0; i < pathSize; i++) {  // used to be ++i


        auto x = m2in(path[i].vector.pose.x); 
        auto y = m2in(path[i].vector.pose.y);
        auto theta = path[i].vector.pose.yaw;
        auto vel = m2in(path[i].vector.vel);

        auto omega = vel / m2in(1/path[i].curvature);

        controller.setTarget(x, y, theta, vel, omega);

        auto output = controller.step(arms::odom::getPosition(), arms::odom::getHeading(true));

        auto linearMotorVelocity = output.linVel/(M_PI*WHEELDIAMETER);
        auto leftRPM = inps2rpm(linearMotorVelocity + output.angVel);
        auto rightRPM = inps2rpm(linearMotorVelocity - output.angVel);


		arms::chassis::leftMotors->move_velocity(leftRPM);
		arms::chassis::rightMotors->move_velocity(rightRPM);
        
        if (i < pathSize - 1) {
            double delay = path[i+1].time - path[i].time;
            pros::delay(delay*1000);
        }
    }
}

void PathFollower::followPathOdom(std::string path_name, std::vector<std::vector<double>> waypoints) {
	std::vector<squiggles::ProfilePoint> path = paths[path_name];
	std::size_t pathSize = path.size();
	for (std::size_t i = 0; i < pathSize; i++) {  // used to be ++i

		bool override = false;

		// if the path x, y coordinates are in waypoints, then set the target
		for (int j = 0; j < waypoints.size(); j++) {
			if (path[i].vector.pose.x == in2m(waypoints[j][0]) && path[i].vector.pose.y == in2m(waypoints[j][1])) {
				arms::Point pos = arms::odom::getPosition();
				// if pos not within 1 inch of target, then move to target
				if (std::abs(pos.x - waypoints[j][0]) > 1 || std::abs(pos.y - waypoints[j][1]) > 1) {
					arms::chassis::move({waypoints[j][0], waypoints[j][1], waypoints[j][2]});
					override = true;
				}
				break;
			}
		}

		if (override) {
			continue;
		}

        auto leftRPM = inps2rpm(m2in(path[i].wheel_velocities[0]));
        auto rightRPM = inps2rpm(m2in(path[i].wheel_velocities[1]));

		arms::chassis::leftMotors->move_velocity(leftRPM);
		arms::chassis::rightMotors->move_velocity(rightRPM);
        
        if (i < pathSize - 1) {
            double delay = path[i+1].time - path[i].time;
            pros::delay(delay*1000);
        }
    }
}
