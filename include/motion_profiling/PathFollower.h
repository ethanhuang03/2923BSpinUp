#include <string>
#include <unordered_map>
#include <vector>
#include "squiggles.hpp"


class PathFollower {
    public:
        double robot_width; // in meters
        void createConstraints(std::string constraint_name, double max_vel, double max_acc, double max_jerk);
        void createPath(std::string constraint_name, std::string path_name, std::vector<std::vector<double>> waypoints);
		void printPath(std::string path_name);
        void followPath(std::string path_name);
        void followPathRamsete(std::string path_name);
		void followPathOdom(std::string path_name, std::vector<std::vector<double>> waypoints);
        PathFollower(double rw) {            
            robot_width = rw;
        }
    private:
        std::unordered_map<std::string, std::vector<squiggles::ProfilePoint>> paths;
        std::unordered_map<std::string, std::vector<double>> constraints;
};