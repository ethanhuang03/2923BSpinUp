#include "motion_profiling/api.h"

void RamseteController::setTarget(double x, double y, double itheta, double ivel, double iomega) { // omega is angular velocity
    // NEED TO Convert To Right Units
    desX = x; // in inch
    desY = y; // in inch
    desT = itheta; // in rad
    velDes = ivel; 
    omegaDes = iomega; 
}

RamseteController::output RamseteController::step(arms::Point point, double itheta) {
    // NEED TO Convert To Right Units
    // Easier to just do the conversion to ROS coordinates here
    double ey = desX - point.x; 
    double ex = desY - point.y; 
    double et = desT - itheta; // in rad
    double ct = M_PI / 2 - itheta; // in rad
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