# Lite3 Gazebo Classic 文档中心

本文档目录包含了 Lite3 Gazebo Classic 仿真环境的完整使用指南和技术文档。

## 文档索引

### 快速开始

| 文档 | 说明 |
|------|------|
| [控制接口规范](./control_interface_spec.md) | 标准控制接口基线 |
| [传感器配置](./sensor_configuration.md) | 传感器配置和使用（**新增**） |
| [键盘控制指南](./keyboard_control_guide.md) | 键盘控制使用说明（**新增**） |
| [故障排除](./troubleshooting.md) | 常见问题及解决方案（**新增**） |
| [FAST-LIVO2 Rcl 分析](./fastlivo2_rcl_analysis.md) | LiDAR-相机外参分析记录 |
| [导航闭环验证记录](./navigation_validation_2026_03_24.md) | FAST-LIVO2 到 Nav2 的最小闭环实际验证过程与结果 |
| [导航当前状态总结](./navigation_status_2026_03_25.md) | 当前已解决问题、已验证能力和下一步计划 |

## 文档更新历史

### 2026-03-02

**新增文档**：
- `sensor_configuration.md` - 整合了传感器配置的完整说明
- `keyboard_control_guide.md` - 键盘控制的详细使用指南
- `troubleshooting.md` - 故障排除指南

**更新内容**：
- Realsense D435i 相机简化为仅 RGB 模式（用于 FAST-LIVO 雷达点着色）
- Livox Mid-360 静态 TF 从 launch 文件移动到 URDF 模型中
- 统一 TF 管理方式，所有 TF 由 `robot_state_publisher` 自动发布

---

## 传感器配置概览

### Livox Mid-360 激光雷达

| 参数 | 值 |
|------|-----|
| 话题 | `/mid360/points`、`/mid360/points_PointCloud2` |
| 发布频率 | 10 Hz |
| 采样点数 | 12000 |
| 探测范围 | 0.1 - 100.0 m |

### Realsense D435i 相机

| 参数 | 值 |
|------|-----|
| 话题 | `/camera/color/image_raw`、`/camera/color/camera_info` |
| 发布频率 | 30 Hz |
| 分辨率 | 1280x720 |
| 视场角 | 87°（水平） |

**注意**：D435i 仅用于为 FAST-LIVO 的雷达点着色，因此只发布 RGB 图像数据。

---

## 键盘控制快速参考

| 按键 | 功能 |
|------|------|
| `1` | 被动模式（Passive） |
| `2` | 收腿模式（FixedDown） |
| `3` | 站立模式（FixedStand） |
| `4` | 小跑模式（Trot） |
| `W/S/A/D` | 前后左右移动 |
| `Q/E` | 旋转控制 |
| `空格` | 停止运动 |

详细控制说明请参考 [键盘控制指南](./keyboard_control_guide.md)。

---

## TF 变换结构

```
base
  └── TORSO
      ├── imu_link
      ├── d435i_link
      │   └── d435i_color_optical_frame
      ├── mid360_base_link
      │   ├── mid360_link
      │   │   └── mid360_livox_laser
      │   └── ...
      └── [四肢关节]
```

**重要**：所有 TF 变换已集成到 URDF 中，由 `robot_state_publisher` 自动发布，无需在 launch 文件中配置静态 TF。

---

## 常用命令

### 启动仿真
```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py
```

### 启动键盘控制
```bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

### 检查话题
```bash
# 传感器话题
ros2 topic list | grep -E "camera|mid360"

# 检查频率
ros2 topic hz /camera/color/image_raw
ros2 topic hz /mid360/points_PointCloud2
```

### 检查 TF
```bash
ros2 run tf2_ros tf2_echo TORSO mid360_livox_laser
ros2 run tf2_ros tf2_echo TORSO d435i_color_optical_frame
```

### 进程清理
```bash
pkill -f gz && pkill -f gazebo && pkill -f ros2
```

---

## 目录结构

```
docs/
├── README.md                 # 本文档（文档索引）
├── sensor_configuration.md   # 传感器配置
├── keyboard_control_guide.md # 键盘控制指南
├── troubleshooting.md        # 故障排除
├── control_interface_spec.md # 控制接口规范
└── fastlivo2_rcl_analysis.md # Rcl 分析记录
