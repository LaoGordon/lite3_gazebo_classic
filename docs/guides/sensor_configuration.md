# 传感器配置文档

本文档详细描述了 Lite3 四足机器人仿真环境中传感器的配置和使用。

## 目录

1. [Realsense D435i 相机配置](#1-realsense-d435i-相机配置仅 rgb)
2. [Livox Mid-360 雷达配置](#2-livox-mid-360 雷达配置)
3. [TF 变换结构](#3-tf 变换结构)
4. [话题列表](#4-话题列表)
5. [常见问题](#5-常见问题)

---

## 1. Realsense D435i 相机配置（仅 RGB）

### 1.1 配置说明

D435i 相机**仅用于为 FAST-LIVO 的雷达点着色**，因此：
- ✅ 只需要 **RGB 彩色图像** 数据
- ❌ 不需要深度图像数据
- ❌ 不需要红外相机数据
- ❌ 不需要点云数据（点云由 Livox 雷达提供）

### 1.2 文件位置

```
src/lite3_gazebo_classic2/lite3_description/xacro/sensors/d435i_sensor.xacro
```

### 1.3 配置参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 水平视场角 | 1.211 rad (87°) | D435i RGB 相机参数 |
| 分辨率 | 1280x720 | RGB 图像分辨率 |
| 发布频率 | 30 Hz | 图像发布频率 |
| 图像格式 | R8G8B8 | RGB888 格式 |
| 安装位置 | xyz="0.18 0.0 0.0525" | 相对于 TORSO 的位置 |

### 1.4 发布的话题

- `/camera/color/image_raw` - RGB 图像消息
- `/camera/color/camera_info` - 相机标定信息

### 1.5 Frame 结构

```
TORSO
  └── d435i_link (物理实体)
      └── d435i_color_optical_frame (光学坐标系)
```

### 1.6 Xacro 配置示例

```xml
<xacro:d435i_sensor name="d435i" parent="TORSO"
                    xyz="0.18 0.0 0.0525" rpy="0 0 0" 
                    topic_ns="camera" visualize="false"/>
```

---

## 2. Livox Mid-360 雷达配置

### 2.1 配置说明

Livox Mid-360 激光雷达提供非重复扫描模式的真实点云数据仿真。

### 2.2 文件位置

```
src/lite3_gazebo_classic2/lite3_description/xacro/sensors/mid360_sensor.xacro
```

### 2.3 配置参数

| 参数 | 值 | 说明 |
|------|-----|------|
| samples | 12000 | 每帧采样点数 |
| downsample | 2 | 降采样倍数 |
| update_rate | 10 Hz | 发布频率 |
| 最大距离 | 100.0 m | 探测范围 |
| 最小距离 | 0.1 m | 探测范围 |

### 2.4 发布的话题

- `/mid360/points` - Livox CustomMsg 格式点云
- `/mid360/points_PointCloud2` - 标准 PointCloud2 格式点云

### 2.5 Frame 结构

```
TORSO
  └── mid360_base_link (雷达底座)
      ├── mid360_link (光学中心)
      │   └── mid360_livox_laser (Livox 插件 frame_id)
```

### 2.6 Xacro 配置示例

```xml
<xacro:mid360_sensor name="mid360" parent="TORSO"
                     xyz="0.0 0.0 0.08" rpy="0 0 0"
                     visualize="true"/>
```

### 2.7 重要说明

**TF 变换已集成到 URDF 中**：
- `mid360_livox_laser` frame 已添加到 URDF
- 通过 `mid360_livox_laser_joint` 零变换连接
- **无需**在 launch 文件中添加静态 TF 节点

---

## 3. TF 变换结构

### 3.1 完整 TF 树

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
      ├── FL_thigh
      │   └── ...
      ├── FR_thigh
      │   └── ...
      ├── HL_thigh
      │   └── ...
      └── HR_thigh
          └── ...
```

### 3.2 TF 发布方式

所有 TF 变换都由 `robot_state_publisher` 从 URDF 中解析并自动发布：
- 无需在 launch 文件中配置静态 TF
- 所有传感器 TF 统一管理
- 便于维护和调试

---

## 4. 话题列表

### 4.1 传感器话题

| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/camera/color/image_raw` | sensor_msgs/msg/Image | RGB 图像 |
| `/camera/color/camera_info` | sensor_msgs/msg/CameraInfo | 相机标定信息 |
| `/mid360/points` | livox_ros_driver2/msg/CustomMsg | Livox 点云（定制格式） |
| `/mid360/points_PointCloud2` | sensor_msgs/msg/PointCloud2 | Livox 点云（标准格式） |

### 4.2 控制话题

| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/cmd_vel` | geometry_msgs/msg/Twist | 速度指令 |
| `/robot_mode` | std_msgs/msg/Int32 | 模式切换指令 |

---

## 5. 常见问题

### 5.1 相机话题不发布

**检查步骤**：
```bash
# 检查话题列表
ros2 topic list | grep camera

# 检查图像频率
ros2 topic hz /camera/color/image_raw

# 检查 TF
ros2 run tf2_ros tf2_echo TORSO d435i_color_optical_frame
```

### 5.2 雷达点云不发布

**检查步骤**：
```bash
# 检查话题列表
ros2 topic list | grep mid360

# 检查点云频率
ros2 topic hz /mid360/points_PointCloud2

# 检查 TF
ros2 run tf2_ros tf2_echo TORSO mid360_livox_laser

# 检查插件路径
echo $GAZEBO_PLUGIN_PATH
```

### 5.3 Gazebo 插件加载失败

**解决方案**：
```bash
# 设置 Gazebo 插件路径
export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH

# 验证插件文件存在
ls $(pwd)/install/ros2_livox_simulation/lib/libros2_livox.so
```

---

## 6. 修改历史

### 2026-03-02
- 简化 D435i 配置为仅 RGB 模式
- 将 Livox 静态 TF 从 launch 文件移动到 URDF
- 统一 TF 管理方式

### 参考文档
- [FAST-LIVO2 Rcl 分析](../references/fastlivo2_rcl_analysis.md)
