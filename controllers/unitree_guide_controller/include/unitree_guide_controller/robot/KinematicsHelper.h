#ifndef KINEMATICS_HELPER_H
#define KINEMATICS_HELPER_H

#include <unitree_guide_controller/common/mathTypes.h>
#include <unitree_guide_controller/robot/RobotLeg.h>
#include <vector>
#include <memory>

/**
 * @brief Helper function to calculate the standard standing foot positions based on model and config.
 * * @param legs List of initialized RobotLeg objects (pointers).
 * @param stand_pos The 12-dimensional joint angles for standing (from yaml).
 * @return Vec34 The calculated 3x4 matrix of foot positions (columns: FR, FL, RR, RL).
 */
Vec34 calculateFeetPosNormalStand(
    const std::vector<std::shared_ptr<RobotLeg>>& legs, 
    const std::vector<double>& stand_pos);

#endif // KINEMATICS_HELPER_H