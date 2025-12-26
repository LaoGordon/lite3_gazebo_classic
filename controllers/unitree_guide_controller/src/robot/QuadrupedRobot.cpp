//
// Created by biao on 24-9-12.
//

#include <iostream>
#include "controller_common/CtrlInterfaces.h"
#include "unitree_guide_controller/robot/QuadrupedRobot.h"
#include "unitree_guide_controller/robot/KinematicsHelper.h"

QuadrupedRobot::QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                               const std::vector<std::string> &feet_names,
                               const std::string &base_name,
                               const std::vector<double> &stand_pos) : ctrl_interfaces_(ctrl_interfaces) {
    KDL::Tree robot_tree;
    kdl_parser::treeFromString(robot_description, robot_tree);

    robot_tree.getChain(base_name, feet_names[0], fr_chain_);
    robot_tree.getChain(base_name, feet_names[1], fl_chain_);
    robot_tree.getChain(base_name, feet_names[2], rr_chain_);
    robot_tree.getChain(base_name, feet_names[3], rl_chain_);


    robot_legs_.resize(4);
    robot_legs_[0] = std::make_shared<RobotLeg>(fr_chain_);
    robot_legs_[1] = std::make_shared<RobotLeg>(fl_chain_);
    robot_legs_[2] = std::make_shared<RobotLeg>(rr_chain_);
    robot_legs_[3] = std::make_shared<RobotLeg>(rl_chain_);

    current_joint_pos_.resize(4);
    current_joint_vel_.resize(4);

    std::cout << "robot_legs_.size(): " << robot_legs_.size() << std::endl;

    // calculate total mass from urdf
    // mass_ = 0;
    // for (const auto &[fst, snd]: robot_tree.getSegments()) {
    //     mass_ += snd.segment.getInertia().getMass();
    // }

    // === [新增] 调用 Helper 动态计算 mass_, pcb_, Ib_ ===
    std::vector<KDL::Chain> chains = {fr_chain_, fl_chain_, rr_chain_, rl_chain_};
    RobotInertialParams params = calculateRobotDynamics(robot_tree, base_name, chains, stand_pos);
    
    this->mass_ = params.mass;
    this->pcb_  = params.pcb;
    this->Ib_   = params.Ib;

    // === [新增] 打印结果方便调试 (可选) ===
    std::cout << "[QuadrupedRobot] Dynamics initialized: Mass=" << mass_ 
              << ", PCB=" << pcb_.transpose() << std::endl;

    // feet_pos_normal_stand_ << 0.184621, 0.184621, -0.164379, -0.164379, -0.157500, 0.157500,
    //         -0.157500, 0.157500, -0.318577, -0.318577, -0.318577, -0.318577;
    feet_pos_normal_stand_ = calculateFeetPosNormalStand(robot_legs_, stand_pos);
    // 打印结果以供验证
    std::cout << "[QuadrupedRobot] Auto-calculated feet_pos_normal_stand_:\n" 
              << feet_pos_normal_stand_ << std::endl;
}

std::vector<KDL::JntArray> QuadrupedRobot::getQ(const std::vector<KDL::Frame> &pEe_list) const {
    std::vector<KDL::JntArray> result;
    result.resize(4);
    for (int i(0); i < 4; ++i) {
        result[i] = robot_legs_[i]->calcQ(pEe_list[i], current_joint_pos_[i]);
    }
    return result;
}

Vec12 QuadrupedRobot::getQ(const Vec34 &vecP) const {
    Vec12 q;
    for (int i(0); i < 4; ++i) {
        KDL::Frame frame;
        frame.p = KDL::Vector(vecP.col(i)[0], vecP.col(i)[1], vecP.col(i)[2]);
        frame.M = KDL::Rotation::Identity();
        q.segment(3 * i, 3) = robot_legs_[i]->calcQ(frame, current_joint_pos_[i]).data;
    }
    return q;
}

Vec12 QuadrupedRobot::getQd(const std::vector<KDL::Frame> &pos, const Vec34 &vel) {
    Vec12 qd;
    const std::vector<KDL::JntArray> q = getQ(pos);
    for (int i(0); i < 4; ++i) {
        Mat3 jacobian = robot_legs_[i]->calcJaco(q[i]).data.topRows(3);
        qd.segment(3 * i, 3) = jacobian.inverse() * vel.col(i);
    }
    return qd;
}

std::vector<KDL::Frame> QuadrupedRobot::getFeet2BPositions() const {
    std::vector<KDL::Frame> result;
    result.resize(4);
    for (int i = 0; i < 4; i++) {
        result[i] = robot_legs_[i]->calcPEe2B(current_joint_pos_[i]);
        result[i].M = KDL::Rotation::Identity();
    }
    return result;
}

KDL::Frame QuadrupedRobot::getFeet2BPositions(const int index) const {
    return robot_legs_[index]->calcPEe2B(current_joint_pos_[index]);
}

KDL::Jacobian QuadrupedRobot::getJacobian(const int index) const {
    return robot_legs_[index]->calcJaco(current_joint_pos_[index]);
}

KDL::JntArray QuadrupedRobot::getTorque(
    const Vec3 &force, const int index) const {
    return robot_legs_[index]->calcTorque(current_joint_pos_[index], force);
}

KDL::JntArray QuadrupedRobot::getTorque(const KDL::Vector &force, int index) const {
    return robot_legs_[index]->calcTorque(current_joint_pos_[index], Vec3(force.data));
}

KDL::Vector QuadrupedRobot::getFeet2BVelocities(const int index) const {
    const Mat3 jacobian = getJacobian(index).data.topRows(3);
    Vec3 foot_velocity = jacobian * current_joint_vel_[index].data;
    return {foot_velocity(0), foot_velocity(1), foot_velocity(2)};
}

std::vector<KDL::Vector> QuadrupedRobot::getFeet2BVelocities() const {
    std::vector<KDL::Vector> result;
    result.resize(4);
    for (int i = 0; i < 4; i++) {
        result[i] = getFeet2BVelocities(i);
    }
    return result;
}

void QuadrupedRobot::update() {
    if (mass_ == 0) return;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray pos_array(3);
        pos_array(0) = ctrl_interfaces_.joint_position_state_interface_[i * 3].get().get_value();
        pos_array(1) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 1].get().get_value();
        pos_array(2) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 2].get().get_value();
        current_joint_pos_[i] = pos_array;

        KDL::JntArray vel_array(3);
        vel_array(0) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3].get().get_value();
        vel_array(1) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 1].get().get_value();
        vel_array(2) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 2].get().get_value();
        current_joint_vel_[i] = vel_array;
    }
}
