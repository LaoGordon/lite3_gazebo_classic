# lite3_gazebo_classic
Lite3 simulation on Gazebo Classic (ROS 2 Humble). Adapted from https://github.com/legubiao/quadruped_ros2_control/.

## 环境要求 (Prerequisites)
* **Ubuntu 22.04 LTS**
* **ROS 2 Humble Hawksbill**
* **Gazebo Classic 11**

## 下载源码 (Download)
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# 将本项目代码克隆到此处 (如果尚未克隆)
git clone https://github.com/LaoGordon/lite3_gazebo_classic.git
```
## 依赖安装 (Dependencies)

推荐使用 rosdep 自动安装所有依赖：

```bash
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
## 编译运行
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py
```
## 打开新终端
```bash
cd ~/ros2_ws
. install/setup.bash
ros2 run keyboard_input keyboard_input 
```
