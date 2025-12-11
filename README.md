# lite3_gazebo_classic
Lite3 simulation on Gazebo Classic (ROS 2 Humble). Adapted from https://github.com/legubiao/quadruped_ros2_control/.

## 📦 依赖安装 (Dependencies)

推荐使用 rosdep 自动安装所有依赖：

```bash
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch lite3_description gazebo_classic.launch.py
