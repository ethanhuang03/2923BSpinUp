#include "squiggles.hpp"
#include "main.h"
#include "ARMS/config.h"

const double MAX_VEL     = 2.0; // in meters per second
const double MAX_ACCEL   = 3.0; // in meters per second per second
const double MAX_JERK    = 6.0; // in meters per second per second per second
const double ROBOT_WIDTH = 0.4; // in meters

squiggles::Constraints constraints = squiggles::Constraints(MAX_VEL, MAX_ACCEL, MAX_JERK);

// Generate Path (waypoint in in, angles in degrees)
std::vector<squiggles::ProfilePoint> create_path(std::vector<std::vector<double>> waypoints) {
    squiggles::SplineGenerator generator = squiggles::SplineGenerator(constraints, std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraints));
    std::vector<squiggles::Pose> points;
    for (int i = 0; i < points.size(); i++) {
        points.push_back(squiggles::Pose(waypoints[i][0]/39.37, waypoints[i][1]/39.37, waypoints[i][2]*M_PI/180)); // convert in to m, deg to rad
    }
    return generator.generate(points); // returns meters, meters, radians
}


class RamseteController {
    public:
        typedef struct {
            double linVel, angVel;
        } output;

        /**
         * b and ζ are tuning parameters where b > 0 and ζ ∈ (0, 1). Larger values of
         * b make convergence more aggressive (like a proportional term), and larger
         * values of ζ provide more damping
         */
        void setGains(double ibeta, double izeta);
        void setTarget(double x, double y, double itheta, double ivel, double iomega); // in inch, in inch, in degree, in inch/s, in rad/s
	    output step(arms::Point point, double itheta); // in inch, in inch, in degree

    private:
	    double beta, zeta, desX, desY, desT, velDes, omegaDes;
};

void RamseteController::setTarget(double x, double y, double itheta, double ivel, double iomega) { // omega is angular velocity
    // NEED TO Convert To Right Units
    desX = x; // in inch
    desY = y; // in inch
    desT = itheta * M_PI/180; // in rad
    velDes = ivel; 
    omegaDes = iomega; 
}

RamseteController::output RamseteController::step(arms::Point point, double itheta) {
    // NEED TO Convert To Right Units
    // Easier to just do the conversion to ROS coordinates here
    double ey = desX - point.x; 
    double ex = desY - point.y; 
    double et = desT - itheta * M_PI/180; // in rad
    double ct = M_PI / 2 - itheta * M_PI/180; // in rad
    ex = cos(-ct) * ex - sin(-ct) * ey;
    ey = sin(-ct) * ex + cos(-ct) * ey;

    double k = 2 * zeta * sqrt(omegaDes * omegaDes + beta * velDes * velDes);
    double vel = velDes * cos(et) + k * ex; // ey in odom coords
    double omega = omegaDes + k * et + beta * velDes * sin(et) * ey / et;

    output out;
    out.linVel = vel;
    out.angVel = omega;
    return out;
}

void RamseteController::setGains(double ib, double izeta) {
    if (ib) {
        beta = ib;
    }
    if (izeta) {
        zeta = izeta;
    }
}


double INPS2RPM(double mps) { // wheel radius in inches
    return (mps * 60 / (2 * M_PI * WHEELRADIUS))/GEARRATIO;
}


void followPath(std::vector<squiggles::ProfilePoint> path) {
    std::size_t pathSize = path.size();
    for (std::size_t i = 0; i < pathSize; ++i) {
        auto leftRPM = INPS2RPM(path[i].wheel_velocities[0]*39.37);
        auto rightRPM = INPS2RPM(path[i].wheel_velocities[1]*39.37);

        arms::chassis::motorMove(arms::chassis::leftMotors, leftRPM, true);
        arms::chassis::motorMove(arms::chassis::rightMotors, rightRPM, true);
        
        if (i < pathSize - 1) {
            double delay = path[i+1].time - path[i].time;
            pros::delay(delay);
        }
    }
}


void followPathRamsete(std::vector<squiggles::ProfilePoint> path) {
    RamseteController controller;
    controller.setGains(0.7, 0.7);

    std::size_t pathSize = path.size();
    for (std::size_t i = 0; i < pathSize; ++i) {
        auto x = path[i].vector.pose.x * 39.37; 
        auto y = path[i].vector.pose.y * 39.37;
        auto theta = path[i].vector.pose.yaw;
        auto vel = path[i].vector.vel;
        // angular velocity in rad/s from left wheel velocity and right wheel velocity in m/s 
        auto omega = 1; // need to be changed

        controller.setTarget(x, y, theta, vel, omega);

        // Constantly update this until delay ends
        auto output = controller.step(arms::odom::getPosition(), arms::odom::getHeading(true));

        auto linearMotorVelocity = output.linVel/(2*M_PI*WHEELRADIUS);
        auto leftRPM = INPS2RPM(linearMotorVelocity + output.angVel);
        auto rightRPM = INPS2RPM(linearMotorVelocity - output.angVel);

        arms::chassis::motorMove(arms::chassis::leftMotors, leftRPM, true);
        arms::chassis::motorMove(arms::chassis::rightMotors, rightRPM, true);
        
        if (i < pathSize - 1) {
            double delay = path[i+1].time - path[i].time;
            pros::delay(delay*1000);
        }
    }
}