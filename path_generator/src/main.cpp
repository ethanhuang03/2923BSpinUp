#include <iostream>
#include <string>
#include <fstream>
#include <unordered_map>
#include <vector>
#include "squiggles.hpp"

#define M_PI 3.14159265358979323846
#define GEARRATIO 1/1    // Gear ratio of chassis motors // output:input 60/36
#define WHEELDIAMETER 3.25 // Diameter of chassis wheels in inches

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

double m2ft(double m) {
	return m * 3.281;
}

double ft2m(double ft) {
	return ft / 3.281;
}

double inps2rpm(double inps) { // wheel radius in inches
    return (inps * 60 / (M_PI * WHEELDIAMETER))/GEARRATIO;
}

double mps2rpm(double mps) { // wheel radius in meters
	return (mps * 60 / (M_PI * in2m(WHEELDIAMETER)))/GEARRATIO;
}

class PathGenerator {
    public:
        double robot_width; // in meters
        void createConstraints(std::string constraint_name, double max_vel, double max_acc, double max_jerk);
        void createPath(std::string constraint_name, std::string path_name, std::vector<std::vector<double>> waypoints);
		void printPath(std::string path_name);
		void interpolateTimes(std::string constraint_name, std::string path_name, std::vector<std::vector<double>> waypoints);
		void generateRyanlibPath(std::string path_name, std::string file_name);
        PathGenerator(double rw) {            
            robot_width = rw;
        }
    private:
        std::unordered_map<std::string, std::vector<squiggles::ProfilePoint>> paths;
        std::unordered_map<std::string, std::vector<double>> constraints;
};

// Constrains in SI Units
void PathGenerator::createConstraints(std::string constraint_name, double max_vel, double max_acc, double max_jerk) { // m/s, m/s^2, m/s^3
    constraints[constraint_name] = {max_vel, max_acc, max_jerk};
}

// Generate Path (waypoint in in, angles in degrees)
void PathGenerator::createPath(std::string constraint_name, std::string path_name, std::vector<std::vector<double>> waypoints) { // in, in, deg 
    std::vector<double> constraint = constraints[constraint_name];
    squiggles::Constraints squiggles_constraints = squiggles::Constraints(constraint[0], constraint[1], constraint[2]);
    squiggles::SplineGenerator generator = squiggles::SplineGenerator(squiggles_constraints, std::make_shared<squiggles::TankModel>(robot_width, squiggles_constraints));
    std::vector<squiggles::Pose> points;
    for (size_t i = 0; i < waypoints.size(); i++) {
        points.push_back(squiggles::Pose(in2m(waypoints[i][0]), in2m(waypoints[i][1]), deg2rad(waypoints[i][2]))); // convert in to m, deg to rad
    }
    paths[path_name] = generator.generate(points); // returns meters, meters, radians
}

// Generate Path (waypoint in in, angles in degrees)
void PathGenerator::interpolateTimes(std::string constraint_name, std::string path_name, std::vector<std::vector<double>> waypoints) { // in, in, deg 
    std::vector<double> constraint = constraints[constraint_name];
    squiggles::Constraints squiggles_constraints = squiggles::Constraints(constraint[0], constraint[1], constraint[2]);
    squiggles::SplineGenerator generator = squiggles::SplineGenerator(squiggles_constraints, std::make_shared<squiggles::TankModel>(robot_width, squiggles_constraints));
    std::vector<squiggles::Pose> points;
    for (size_t i = 0; i < waypoints.size(); i++) {
        points.push_back(squiggles::Pose(in2m(waypoints[i][0]), in2m(waypoints[i][1]), deg2rad(waypoints[i][2]))); // convert in to m, deg to rad
    }
	
    std::vector<squiggles::ProfilePoint> temp =  generator.generate(points); // returns meters, meters, radians
	double time = temp[temp.size()-1].time;
	double counter = 0;
	while (counter <= time) {
		paths[path_name].push_back(generator.get_point_at_time(temp[0].vector, temp[temp.size()-1].vector, temp, counter));
		counter += 0.01;
	}
}

void PathGenerator::generateRyanlibPath(std::string path_name, std::string file_name) {
	std::vector<squiggles::ProfilePoint> path = paths[path_name];
	// {left_pos, right_pos, left_vel, right_vel, left_acc, right_acc}
	double prev_time = 0;
	std::vector<std::vector<double>> trajectory = {{0,0,0,0,0,0}};
	for (size_t i = 1; i < path.size()-1; i++) {
		double left_distance = trajectory[trajectory.size()-1][0] + m2ft(path[i].wheel_velocities[0]) * (path[i+1].time - path[i].time);
		double right_distance = trajectory[trajectory.size()-1][1] + m2ft(path[i].wheel_velocities[1]) * (path[i+1].time - path[i].time);
		double left_accel = (m2ft(path[i].wheel_velocities[0]) - trajectory[trajectory.size()-1][2]) / (path[i].time - prev_time);
		double right_accel = (m2ft(path[i].wheel_velocities[1]) - trajectory[trajectory.size()-1][3]) / (path[i].time - prev_time);
		prev_time = path[i].time;
		trajectory.push_back({left_distance, right_distance, m2ft(path[i].wheel_velocities[0]), m2ft(path[i].wheel_velocities[1]), left_accel, right_accel});
	}

	// last point
	trajectory.push_back({
		trajectory[trajectory.size()-1][0] + m2ft(path[path.size()-1].wheel_velocities[0]) * (0.01),
		trajectory[trajectory.size()-1][1] + m2ft(path[path.size()-1].wheel_velocities[1]) * (0.01),
		m2ft(path[path.size()-1].wheel_velocities[0]),
		m2ft(path[path.size()-1].wheel_velocities[1]),
		(m2ft(path[path.size()-1].wheel_velocities[0]) - trajectory[trajectory.size()-1][2]) / (path[path.size()-1].time - prev_time),
		(m2ft(path[path.size()-1].wheel_velocities[1]) - trajectory[trajectory.size()-1][3]) / (path[path.size()-1].time - prev_time)
	});

    std::ofstream file;
    file.open(file_name+".cpp", std::ios::out | std::ios::app);
    file << "Trajectory " << file_name << "::" << path_name << " = {" << std::endl;
    for (size_t i = 0; i < trajectory.size(); i++) {
        file << "    { ";
        for (size_t j = 0; j < trajectory[i].size(); j++) {
            file << trajectory[i][j];
            if (j < trajectory[i].size() - 1) {
                file << ", ";
            }
        }
        file << " }";
        if (i < trajectory.size() - 1) {
            file << ",";
        }
        file << std::endl;
    }
    file << "};" << std::endl;
	file << std::endl;
	file << std::endl;
    file.close();
}

void PathGenerator::printPath(std::string path_name) {
	std::vector<squiggles::ProfilePoint> path = paths[path_name];
	std::cout << "|x|y|theta|v|a|j|w-rpm|k|t|" << std::endl;
	for (size_t i = 0; i < path.size(); i++) {
		std::cout << "|" << m2in(path[i].vector.pose.x) << "|" << m2in(path[i].vector.pose.y) << "|" << rad2deg(path[i].vector.pose.yaw) << "|" << path[i].vector.vel << "|" << path[i].vector.accel << "|" << path[i].vector.jerk << "|" << inps2rpm(m2in(path[i].wheel_velocities[0])) << ", " << inps2rpm(m2in(path[i].wheel_velocities[1])) << "|" << path[i].curvature << "|" << path[i].time << "|" << std::endl;
	}	
}


int main()
{
	PathGenerator pathgenerator(0.37465);
	pathgenerator.createConstraints("test", 0.275666666666, 9, 18);
	pathgenerator.interpolateTimes("test", "test_path", {{48, 72, -90}, {48, 48, -90}});
	pathgenerator.generateRyanlibPath("test_path", "skills");
}