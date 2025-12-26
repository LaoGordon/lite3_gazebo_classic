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