# Lite3 Gazebo Classic 仿真

Lite3 四足机器人的 Gazebo Classic 仿真环境，支持 ROS 2 Humble。

## 快速开始

### 1. 安装依赖

```bash
sudo apt update
sudo apt install \
  libgoogle-glog-dev \
  ros-humble-topic-tools \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup

cd ~/quadruped_ws
rosdep install --from-paths src --ignore-src -r -y
git submodule update --init --recursive
```

说明：

- `ros-humble-topic-tools` 是启动 `gazebo_classic.launch.py` 时必需的运行依赖，用于 `topic_tools/relay` 节点将 `/imu_sensor_broadcaster/imu` 转发到 `/livox/imu`
- `ros-humble-navigation2` 与 `ros-humble-nav2-bringup` 是静态地图验证与 Nav2 导航链所需依赖，其中包含 `nav2_map_server`、`nav2_lifecycle_manager` 与 `nav2_bringup`

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

在仿真启动后，可在另一个终端启动 FAST-LIVO2。

默认启动：

```bash
cd ~/quadruped_ws
source install/setup.bash
ros2 launch fast_livo mapping_gazebo.launch.py
```

常用启动参数：

- `use_rviz:=true|false`
  - 是否同时启动 RViz2
  - 默认值为 `false`
- `enable_pcd_save:=true|false`
  - 是否启用 FAST-LIVO2 原生 PCD 落盘
  - 默认值为 `false`
  - 仅在一次性建图、需要导出全局点云时开启

示例：

仅启动 FAST-LIVO2，不开 RViz，不保存 PCD：

```bash
ros2 launch fast_livo mapping_gazebo.launch.py
```

启动 FAST-LIVO2 并打开 RViz：

```bash
ros2 launch fast_livo mapping_gazebo.launch.py use_rviz:=true
```

启动 FAST-LIVO2 并导出全局 PCD：

```bash
ros2 launch fast_livo mapping_gazebo.launch.py enable_pcd_save:=true
```

同时打开 RViz 并导出全局 PCD：

```bash
ros2 launch fast_livo mapping_gazebo.launch.py use_rviz:=true enable_pcd_save:=true
```

启用 `enable_pcd_save:=true` 后，FAST-LIVO2 会在退出时输出全局点云到：

- `src/lite3_gazebo_classic/src/FAST-LIVO2/Log/PCD/all_raw_points.pcd`
- `src/lite3_gazebo_classic/src/FAST-LIVO2/Log/PCD/all_downsampled_points.pcd`

当前仿真链路中：

- 激光雷达输入来自 Livox 仿真插件
- IMU 输入由控制器广播并映射到 FAST-LIVO2 所需话题
- 相机输入来自仿真 D435i RGB 相机

Rcl 外参分析与结论见：

- [FAST-LIVO2 Rcl 分析](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/references/fastlivo2_rcl_analysis.md)
- [FAST-LIVO2 原生 PCD 落盘说明](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/planning/fastlivo2_pcd_export_guide_2026_03_31.md)

### 6. 一键导航验证

若你已经生成并保存了静态地图，可直接使用仓库内的一键脚本启动整套验证链。

默认入口：

```bash
cd ~/quadruped_ws/src/lite3_gazebo_classic
./run_navigation_validation.sh
```

该脚本会按顺序启动：

- `run_gazebo_world.sh`
- FAST-LIVO2
- `fastlivo_nav_bridge`
- `static_map.launch.py`
- 临时 `map -> odom` 静态 TF
- `navigation_main.launch.py`
- 导航 RViz

默认使用的静态地图为：

- `navigation/quadruped_nav_bringup/maps/floor_small_test.yaml`

若要指定其他 world，可将 world 文件路径作为第一个参数传入：

```bash
cd ~/quadruped_ws/src/lite3_gazebo_classic
./run_navigation_validation.sh /absolute/path/to/your.world
```

若要调整阶段间等待时间，可通过环境变量修改：

```bash
cd ~/quadruped_ws/src/lite3_gazebo_classic
GAZEBO_DELAY=12 FASTLIVO_DELAY=8 BRIDGE_DELAY=5 STATIC_MAP_DELAY=4 NAV2_DELAY=5 ./run_navigation_validation.sh
```

若不需要自动打开 RViz，可关闭对应窗口：

```bash
cd ~/quadruped_ws/src/lite3_gazebo_classic
FASTLIO_RVIZ=false NAV_RVIZ=false ./run_navigation_validation.sh
```

## 标准控制接口

当前稳定的外部控制接口为：

- `/cmd_vel`
- `/robot_mode`

详细约定见：

- [控制接口规范](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/references/control_interface_spec.md)

## 文档导航

- [文档索引](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/README.md)
- [控制接口规范](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/references/control_interface_spec.md)
- [键盘控制指南](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/guides/keyboard_control_guide.md)
- [传感器配置](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/guides/sensor_configuration.md)
- [FAST-LIVO2 Rcl 分析](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/references/fastlivo2_rcl_analysis.md)
- [故障排除](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/guides/troubleshooting.md)

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11 Classic
