#!/usr/bin/env bash

set -euo pipefail

echo "[1/5] Checking required topics"
ros2 topic list | rg '^/odom$|^/obstacle_points$|^/floor_map$|^/cmd_vel$|^/cmd_vel_nav$' || true

echo
echo "[2/5] Checking TF chain availability"
timeout 5 ros2 run tf2_ros tf2_echo map odom >/tmp/nav_tf_map_odom.log 2>&1 || true
timeout 5 ros2 run tf2_ros tf2_echo odom base >/tmp/nav_tf_odom_base.log 2>&1 || true
sed -n '1,20p' /tmp/nav_tf_map_odom.log || true
sed -n '1,20p' /tmp/nav_tf_odom_base.log || true

echo
echo "[3/5] Checking Nav2 lifecycle nodes"
ros2 lifecycle nodes || true

echo
echo "[4/5] Inspecting map and velocity topics"
timeout 3 ros2 topic echo /floor_map --once || true
timeout 3 ros2 topic echo /cmd_vel_nav --once || true

echo
echo "[5/5] If the topics above exist, send a goal from RViz or use:"
echo "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'"
