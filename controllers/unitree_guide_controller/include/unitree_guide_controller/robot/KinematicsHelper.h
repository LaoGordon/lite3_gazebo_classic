#ifndef KINEMATICS_HELPER_H
#define KINEMATICS_HELPER_H

#include <unitree_guide_controller/common/mathTypes.h>
#include <unitree_guide_controller/robot/RobotLeg.h>

#include <vector>
#include <memory>
#include <iostream>
#include <string>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/rigidbodyinertia.hpp>

// 定义结构体存储整机动力学参数
struct RobotInertialParams {
    double mass;
    Vec3 pcb;   // 质心位置 (Position of Center of Body)
    Mat3 Ib;    // 整机转动惯量 (Inertia of Body)
};

/**
 * @brief Helper function to calculate the standard standing foot positions based on model and config.
 * * @param legs List of initialized RobotLeg objects (pointers).
 * @param stand_pos The 12-dimensional joint angles for standing (from yaml).
 * @return Vec34 The calculated 3x4 matrix of foot positions (columns: FR, FL, RR, RL).
 */
Vec34 calculateFeetPosNormalStand(
    const std::vector<std::shared_ptr<RobotLeg>>& legs, 
    const std::vector<double>& stand_pos);

// 新增：计算整机动力学参数
RobotInertialParams calculateRobotDynamics(
    const KDL::Tree &robot_tree, 
    const std::string &base_name,
    const std::vector<KDL::Chain> &leg_chains, 
    const std::vector<double> &stand_pos);

#endif // KINEMATICS_HELPER_H