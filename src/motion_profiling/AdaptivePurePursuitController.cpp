#include <string>
#include <unordered_map>
#include <vector>
#include "squiggles.hpp"
#include "ARMS/api.h"
#include "ARMS/config.h"
#include "motion_profiling/RamseteController.h"

// input: in, in, degree
// squiggles: m, m, rad
// output: in, in, rad

std::unordered_map<std::string, std::vector<squiggles::ProfilePoint>> paths; // m, m, rad
std::unordered_map<std::string, std::vector<std::vector<double>>> waypoint_storage; // in, in, rad
std::unordered_map<std::string, std::vector<double>> constraints;
RamseteController ramsete;
double ROBOT_WIDTH = 0.5; // in meters
double LOOKAHEAD = 1; // in meters // Distance to look ahead on path
double WAYPOINT_THRESHOLD = 0.1; // in meters // Distance at which to switch to Ramsete controller

struct Point {
  double x; // in inch
  double y; // in inch
};

struct Pose {
	Point position;
	double heading; // in radians
};

double inps2rpm(double inps) { // wheel radius in inches
    return (inps * 60 / (M_PI * WHEELDIAMETER))*GEARRATIO;
}

double m2in(double m) {
    return m * 39.37;
}

double in2m(double in) {
    return in / 39.37;
}


// Constrains in SI Units
void createConstraints(std::string constraint_name, double max_vel, double max_acc, double max_jerk) {
    constraints[constraint_name] = {max_vel, max_acc, max_jerk};
}

// Generate Path (waypoint in in, angles in degrees)
void createPath(std::string constraint_name, std::string path_name, std::vector<std::vector<double>> waypoints) {
    std::vector<double> constraint = constraints[constraint_name];
    squiggles::Constraints squiggles_constraints = squiggles::Constraints(constraint[0], constraint[1], constraint[2]);
    squiggles::SplineGenerator generator = squiggles::SplineGenerator(squiggles_constraints, std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraints));
    std::vector<squiggles::Pose> points;
	std::vector<std::vector<double>> storage;
    for (int i = 0; i < waypoints.size(); i++) {
        points.push_back(squiggles::Pose(in2m(waypoints[i][0]), in2m(waypoints[i][1]), waypoints[i][2]*M_PI/180)); // convert in to m, deg to rad
		storage.push_back({waypoints[i][0], waypoints[i][1], waypoints[i][2]*M_PI/180});
    }
    paths[path_name] = generator.generate(points); // returns meters, meters, radians
	waypoint_storage[path_name] = storage;
}


Point calcLookaheadPoint(Point rpoint, std::string path_name) {
	std::vector<squiggles::ProfilePoint> path = paths[path_name];
	Point closest_point = {m2in(path[0].vector.pose.x), m2in(path[0].vector.pose.y)};
	double closest_distance = std::sqrt(std::pow(rpoint.x - closest_point.x, 2) + std::pow(rpoint.y - closest_point.y, 2));

	for (int i = 1; i < path.size(); i++) {
		Point point = {m2in(path[i].vector.pose.x), m2in(path[i].vector.pose.y)};
    	double distance = std::sqrt(std::pow(rpoint.x - point.x, 2) + std::pow(rpoint.y - point.y, 2));
		if (distance < closest_distance) {
			closest_point = point;
			closest_distance = distance;
		}
	}

	Point lookahead_point = closest_point;
	for (int i = 0; i < path.size(); i++) {
		Point point = {m2in(path[i].vector.pose.x), m2in(path[i].vector.pose.y)};

		double distance = std::sqrt(std::pow(closest_point.x - point.x, 2) + std::pow(closest_point.y - point.y, 2));
		if (distance > LOOKAHEAD) {
			lookahead_point = point;
			break;
		}
	}
	return lookahead_point;
}

void followPath(std::string path_name){
	std::vector<squiggles::ProfilePoint> path = paths[path_name];
	std::vector<std::vector<double>> waypoints = waypoint_storage[path_name];
	
}
