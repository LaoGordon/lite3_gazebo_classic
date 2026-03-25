# Lite3 Gazebo Classic 仿真

Lite3 四足机器人的 Gazebo Classic 仿真环境，支持 ROS 2 Humble。

## 快速开始

### 1. 安装依赖

```bash
sudo apt update
sudo apt install libgoogle-glog-dev

cd ~/quadruped_ws
rosdep install --from-paths src --ignore-src -r -y
git submodule update --init --recursive
```

### 2. 编译

```bash
cd ~/quadruped_ws
colcon build --symlink-install
```

### 3. 启动仿真

默认空场景：

```bash
./run_gazebo.sh
```

加载仓库内置 world（默认加载 `quadruped_playground/worlds/test_world.world`）：

```bash
./run_gazebo_world.sh
```

加载自定义 Gazebo Classic world：

```bash
./run_gazebo_world.sh /absolute/path/to/your.world
```

手动方式：

```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$(pwd)/src/lite3_gazebo_classic/libraries/quadruped_playground/models:$GAZEBO_MODEL_PATH
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py world:=$(pwd)/src/lite3_gazebo_classic/libraries/quadruped_playground/worlds/test_world.world
```

### 4. 启动主链遥操作

```bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

主链模式键：

- `1`：`PASSIVE`
- `2`：`FIXEDDOWN`
- `3`：`FIXEDSTAND`
- `4`：`TROTTING`

运动控制键：

- `W/S`：前后
- `A/D`：横移
- `Q/E`：偏航
- `空格`：清零当前运动指令

### 5. FAST-LIVO2 使用

在仿真启动后，可在另一个终端启动 FAST-LIVO2：

```bash
cd ~/quadruped_ws
source install/setup.bash
ros2 launch fast_livo mapping_gazebo.launch.py use_rviz:=True
```

当前仿真链路中：

- 激光雷达输入来自 Livox 仿真插件
- IMU 输入由控制器广播并映射到 FAST-LIVO2 所需话题
- 相机输入来自仿真 D435i RGB 相机

Rcl 外参分析与结论见：

- [FAST-LIVO2 Rcl 分析](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/fastlivo2_rcl_analysis.md)

## 标准控制接口

当前稳定的外部控制接口为：

- `/cmd_vel`
- `/robot_mode`

详细约定见：

- [控制接口规范](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/control_interface_spec.md)

## 文档导航

- [文档索引](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/README.md)
- [控制接口规范](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/control_interface_spec.md)
- [键盘控制指南](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/keyboard_control_guide.md)
- [传感器配置](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/sensor_configuration.md)
- [FAST-LIVO2 Rcl 分析](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/fastlivo2_rcl_analysis.md)
- [故障排除](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/troubleshooting.md)

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11 Classic
