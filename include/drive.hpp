#pragma once
#include "main.h"

/**
 * @brief Moves for set amount of time at set speed
 * 
 * @param speed speed [-1, 1]
 * @param time time
 */
void moveTime(std::pair<double, double> speed, QTime time);

/**
 * @brief Moves for set amount of time at set speed. Heading is maintained using PID
 * 
 * @param speed speed [-1, 1]
 * @param time time
 */
void moveTimeHeadingCorrect(double speed, QTime time);

/**
 * @brief Moves the robot for the desired distance (using closed-loop control)
 * 
 * @param target desired location
 * @param time maximum time to run before ending the loop, defaulted to infinite
 */
void moveDistance(QLength target, QTime time = 2_min);

/**
 * @brief Turns the robot to the desired global angle (using closed-loop control)
 * 
 * @param targetAngle - the target odometry global angle to turn to, normalized to [-pi, pi]
 */
void turnToAngle(QAngle targetAngle, QTime time = 2_min);