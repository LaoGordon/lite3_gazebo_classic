#include "unitree_guide_controller/robot/KinematicsHelper.h"
#include <iostream>

Vec34 calculateFeetPosNormalStand(
    const std::vector<std::shared_ptr<RobotLeg>>& legs, 
    const std::vector<double>& stand_pos) 
{
    Vec34 feet_pos;
    feet_pos.setZero();

    // Safety checks
    if (legs.size() != 4) {
        std::cerr << "[KinematicsHelper] Error: Robot must have 4 legs to calculate stand pose." << std::endl;
        return feet_pos;
    }
    if (stand_pos.size() != 12) {
        std::cerr << "[KinematicsHelper] Error: stand_pos vector must have size 12." << std::endl;
        return feet_pos;
    }

    // Iterate through all 4 legs (FR, FL, RR, RL)
    for (int i = 0; i < 4; ++i) {
        KDL::JntArray q_stand(3);
        
        // Map linear index to leg joints (3 joints per leg)
        // stand_pos format: [FR_0, FR_1, FR_2, FL_0, ..., RL_2]
        q_stand(0) = stand_pos[i * 3 + 0];
        q_stand(1) = stand_pos[i * 3 + 1];
        q_stand(2) = stand_pos[i * 3 + 2];

        // Perform Forward Kinematics for this leg
        KDL::Frame foot_pose = legs[i]->calcPEe2B(q_stand);

        // Fill the result matrix column
        feet_pos.col(i) << foot_pose.p.x(), 
                           foot_pose.p.y(), 
                           foot_pose.p.z();
    }

    return feet_pos;
}

RobotInertialParams calculateRobotDynamics(
    const KDL::Tree &robot_tree, 
    const std::string &base_name,
    const std::vector<KDL::Chain> &leg_chains, 
    const std::vector<double> &stand_pos)
{
    RobotInertialParams params;
    params.mass = 0.0;
    params.pcb.setZero();
    params.Ib.setZero();

    // 1. 初始化惯量累加器
    KDL::RigidBodyInertia total_inertia;

    // 2. 获取 Base Link (躯干) 的惯量
    auto base_segment_it = robot_tree.getSegment(base_name);
    if(base_segment_it != robot_tree.getSegments().end()){
        total_inertia = total_inertia + base_segment_it->second.segment.getInertia();
    } else {
        std::cerr << "[KinematicsHelper] Error: Base link '" << base_name << "' not found in tree!" << std::endl;
    }

    // 3. 遍历四条腿 (假设 leg_chains 顺序为 FR, FL, RR, RL)
    // 检查维度是否匹配
    if(leg_chains.size() != 4) {
         std::cerr << "[KinematicsHelper] Error: 4 leg chains required." << std::endl;
         return params;
    }
    
    // 假设 stand_pos 包含了所有活动关节的角度。
    // 为了更通用，这里假设每条腿的活动关节数等于 chain 中的 segment 数（减去固定关节）
    // 简单起见，我们按顺序取 stand_pos。需确保 stand_pos 顺序为 [Leg0_J0, Leg0_J1, ..., Leg1_J0, ...]
    
    int joint_idx_counter = 0;

    for(int i=0; i<4; ++i) {
        KDL::Frame frame_accum = KDL::Frame::Identity(); // 累积变换矩阵 (从 Base 到当前 Segment)
        
        const KDL::Chain& current_chain = leg_chains[i];
        
        for(unsigned int j=0; j < current_chain.getNrOfSegments(); ++j) {
            auto segment = current_chain.getSegment(j);
            KDL::Joint joint = segment.getJoint();

            // 更新变换矩阵
            if(joint.getType() == KDL::Joint::None) {
                frame_accum = frame_accum * segment.pose(0);
            } else {
                // 是活动关节，从 stand_pos 取值
                if(joint_idx_counter < (int)stand_pos.size()) {
                    double q = stand_pos[joint_idx_counter];
                    frame_accum = frame_accum * segment.pose(q);
                    joint_idx_counter++;
                } else {
                    std::cerr << "[KinematicsHelper] Error: stand_pos size mismatch!" << std::endl;
                }
            }
            
            // 将该连杆的惯量变换到基座坐标系并累加
            // segment.getInertia() 是在连杆局部坐标系下的惯量
            // RefFrame(frame_accum) 将其变换到 Base 坐标系
            total_inertia = total_inertia + (frame_accum * segment.getInertia());
        }
    }

    // 4. 提取结果
    // 总质量
    params.mass = total_inertia.getMass();
    
    // 质心位置 (相对于 Base)
    KDL::Vector com = total_inertia.getCOG();
    params.pcb << com.x(), com.y(), com.z();

    // 关于质心的转动惯量
    // RefPoint(com) 将参考点移动到质心
    KDL::RotationalInertia I_com = total_inertia.RefPoint(com).getRotationalInertia();
    
    // KDL 存储顺序是 Row-Major 还是 Column-Major 需要注意，但是对称矩阵通常直接映射
    // KDL::RotationalInertia 数据访问：data[0-8]
    params.Ib << I_com.data[0], I_com.data[1], I_com.data[2],
                 I_com.data[3], I_com.data[4], I_com.data[5],
                 I_com.data[6], I_com.data[7], I_com.data[8];
                 
    return params;
}