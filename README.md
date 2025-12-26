Lite3 simulation on Gazebo Classic (ROS 2 Humble). Adapted from https://github.com/legubiao/quadruped_ros2_control/.

## 系统要求 (System Requirements)

### 环境依赖
- **Ubuntu 22.04** 
- **ROS 2 Humble** 
- **Gazebo Classic 11**

## 工作空间设置 (Workspace Setup)

### 1. 创建工作空间
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. 克隆项目
```bash
git clone https://github.com/LaoGordon/lite3_gazebo_classic.git
```

## 依赖安装 (Dependencies)

### 推荐使用 rosdep 自动安装所有依赖：
```bash
cd ~/ros2_ws
sudo apt update

# 安装项目依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 额外依赖（如果rosdep无法安装）
```bash
# Gazebo仿真依赖
sudo apt install gazebo libgazebo-dev
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs

# ROS 2控制依赖
sudo apt install ros-${ROS_DISTRO}-ros2-control
sudo apt install ros-${ROS_DISTRO}-ros2-controllers
sudo apt install ros-${ROS_DISTRO}-joint-state-publisher-gui
```

## 编译运行 (Build and Run)

### 构建项目
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 配置环境
```bash
source ~/ros2_ws/install/setup.bash
```

### 启动Gazebo Classic仿真
```bash
ros2 launch lite3_description gazebo_classic.launch.py
```

## 控制机器人 (Robot Control)

### 键盘控制 (Keyboard Control)
打开新终端：
```bash
cd ~/ros2_ws
. install/setup.bash
ros2 run keyboard_input keyboard_input
```
## 四足机器人参数调整 
参数调整一般在const.xacro中调整
查看整体的urdf文件：
```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash
ros2 run xacro xacro src/lite3_gazebo_classic/lite3_description/xacro/robot.xacro GAZEBO:=true CLASSIC:=true > src/lite3_gazebo_classic/lite3_description/urdf/robot.urdf
```
