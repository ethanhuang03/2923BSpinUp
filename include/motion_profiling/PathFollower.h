#include <string>
#include <unordered_map>
#include <vector>
#include "squiggles.hpp"


class PathFollower {
    public:
        double robot_width; // in meters
        void createConstraints(std::string constraint_name, double max_vel, double max_acc, double max_jerk);
        void createPath(std::string constraint_name, std::string path_name, std::vector<std::vector<double>> waypoints);
        void followPath(std::string path_name);
        void followPathRamsete(std::string path_name);
        PathFollower(double robot_width) {            
            robot_width = robot_width;
        }
    private:
        std::unordered_map<std::string, std::vector<squiggles::ProfilePoint>> paths;
        std::unordered_map<std::string, std::vector<double>> constraints;
};