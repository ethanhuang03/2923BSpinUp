#include "main.h"
#include "squiggles.hpp"
/*
struct inputPoint{
	okapi::QLength x;
	okapi::QLength y; 
	okapi::QAngle heading;
};

squiggles::Constraints constraints = squiggles::Constraints(constraint.maxVelocity.convert(mps), constraint.maxAcceleration.convert(mps2), constraint.maxJerk.convert(mps3));
squiggles::SplineGenerator generator = squiggles::SplineGenerator(constraints, std::make_shared<squiggles::TankModel>(chassis->getChassisScales().wheelTrack.convert(meter), constraints));


std::vector<inputPoint> points = {
	{48_in, 120_in, -90_deg},
	{48_in, 96_in, -90_deg}
};


void generatePath(std::vector<inputPoint> points) {
	std::vector<squiggles::Pose> poses; 
	for (size_t i = 0; i < points.size(); i++) {
		poses.push_back(squiggles::Pose(points[i].x.convert(meter), points[i].y.convert(meter), points[i].heading.convert(radian)));
	}
	std::vector<squiggles::ProfilePoint> path = generator.generate(poses);

	// {left_pos, right_pos, left_vel, right_vel, left_acc, right_acc}
	double prev_time = 0;
	std::vector<std::vector<double>> trajectory = {{0,0,0,0,0,0}};
	for (size_t i = 0; i < path.size()-1; i++) {
		double left_distance = trajectory[trajectory.size()-1][0] + path[i].wheel_velocities[0] * (path[i+1].time - path[i].time);
		double right_distance = trajectory[trajectory.size()-1][1] + path[i].wheel_velocities[1] * (path[i+1].time - path[i].time);
		double left_accel = (path[i].wheel_velocities[0] - trajectory[trajectory.size()-1][2]) / (path[i].time - prev_time);
		double right_accel = (path[i].wheel_velocities[1] - trajectory[trajectory.size()-1][3]) / (path[i].time - prev_time);
		prev_time = path[i].time;
		trajectory.push_back({left_distance, right_distance, path[i].wheel_velocities[0], path[i].wheel_velocities[1], left_accel, right_accel});
	}
}
*/



