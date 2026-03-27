# 故障排除指南

本文档收集了 Lite3 Gazebo Classic 仿真环境中的常见问题及解决方案。

## 目录

1. [Gazebo 相关问题](#1-gazebo 相关问题)
2. [Livox 雷达相关问题](#2-livox 雷达相关问题)
3. [传感器配置问题](#3-传感器配置问题)
4. [进程管理](#4-进程管理)

---

## 1. Gazebo 相关问题

### 1.1 Gazebo 服务器崩溃（exit code -11）

**问题现象**：
```
[ERROR] [gzserver-3]: process has died [pid XXX, exit code -11, cmd 'gzserver ...']
```

**原因**：Livox 插件与 Gazebo 传感器噪声配置不兼容

**解决方案**：
- 确保使用最新版本的代码
- `mid360_sensor.xacro` 文件中已移除噪声配置部分

### 1.2 "[Err] [Sensor.cc:510] Get noise index not valid" 错误

**问题现象**：
```
[gzserver-3] [Err] [Sensor.cc:510] Get noise index not valid
```

**原因**：Gazebo 11 中的传感器噪声配置格式与 Livox 插件不兼容

**解决方案**：
- 已在 `mid360_sensor.xacro` 中移除 `<noise>` 配置块
- 此警告不影响 Livox 插件正常工作

### 1.3 Gazebo 端口被占用

**问题现象**：
```
[Err] [Master.cc:96] EXCEPTION: Unable to start server[bind: Address already in use]
```

**解决方案**：
```bash
# 清理 Gazebo 进程
pkill -f gzserver
pkill -f gzclient
pkill -f gazebo

# 清理端口占用
lsof -ti:11345 | xargs kill -9
```

---

## 2. Livox 雷达相关问题

### 2.1 Livox 插件无法加载

**问题现象**：
- Livox 点云数据未发布
- RVIZ 中看不到点云

**解决方案**：

1. 确保正确设置了 Gazebo 插件路径：
   ```bash
   export GAZEBO_PLUGIN_PATH=$(pwd)/install/ros2_livox_simulation/lib:$GAZEBO_PLUGIN_PATH
   ```

2. 验证插件文件存在：
   ```bash
   ls $(pwd)/install/ros2_livox_simulation/lib/libros2_livox.so
   ```

3. 检查插件编译状态：
   ```bash
   cd ~/quadruped_ws
   colcon build --packages-select ros2_livox_simulation
   ```

### 2.2 验证 Livox 功能

```bash
# 查看 Livox 点云话题
ros2 topic list | grep mid360

# 查看点云数据
ros2 topic echo /mid360/points --no-arr | head -20

# 检查点云频率
ros2 topic hz /mid360/points_PointCloud2
```

### 2.3 RVIZ 中显示点云

1. 启动 RVIZ：`ros2 run rviz2 rviz2`
2. 添加 PointCloud2 显示
3. 设置 Topic 为：`/mid360/points`
4. 设置 Fixed Frame 为：`base`

---

## 3. 传感器配置问题

### 3.1 相机话题不发布

**检查步骤**：
```bash
# 检查话题列表
ros2 topic list | grep camera

# 检查图像频率
ros2 topic hz /camera/color/image_raw

# 检查 TF
ros2 run tf2_ros tf2_echo TORSO d435i_color_optical_frame
```

### 3.2 雷达点云不发布

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

### 3.3 TF 变换问题

**检查 TF 树**：
```bash
ros2 run tf2_tools view_frames.py
evince frames.pdf
```

**检查特定 TF**：
```bash
ros2 run tf2_ros tf2_echo TORSO mid360_link
ros2 run tf2_ros tf2_echo TORSO d435i_color_optical_frame
```

---

## 4. 进程管理

### 4.1 完整进程清理方案

```bash
# 方法 1：快速清理所有相关进程（推荐）
pkill -f gz && pkill -f gazebo && pkill -f ros2

# 方法 2：逐步清理（更安全）
# 终止 Gazebo 服务器和客户端
pkill -f gzserver
pkill -f gzclient
# 终止 ROS2 相关进程
pkill -f ros2
# 清理可能的残留进程
pkill -f rviz2
pkill -f robot_state_publisher

# 方法 3：强制清理（当其他方法无效时）
sudo pkill -9 -f gz
sudo pkill -9 -f gazebo
sudo pkill -9 -f ros2

# 清理 Gazebo 缓存和临时文件
rm -rf ~/.gazebo
rm -rf /tmp/gazebo-*
```

### 4.2 预防措施

- 每次运行仿真前先执行清理命令
- 使用 `Ctrl+C` 正常终止仿真，避免直接关闭终端
- 如果仿真卡死，先尝试 `ros2 lifecycle set` 命令，再进行进程清理

### 4.3 检查进程状态

```bash
# 查看 ROS2 节点
ros2 node list

# 查看生命周期节点状态
ros2 lifecycle list

# 查看话题
ros2 topic list

# 查看正在运行的进程
ps aux | grep -E "gazebo|gzserver|gzclient|ros2"
```

---

## 5. 编译问题

### 5.1 依赖缺失

```bash
# 安装系统依赖
sudo apt update
sudo apt install liblivox-sdk2-dev libgoogle-glog-dev

# 安装 ROS2 依赖
cd ~/quadruped_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5.2 子模块未初始化

```bash
# 初始化子模块
cd ~/quadruped_ws/src/lite3_gazebo_classic2
git submodule update --init --recursive
```

### 5.3 清理重新编译

```bash
# 清理构建缓存
cd ~/quadruped_ws
rm -rf build/ install/ log/

# 重新编译
colcon build --symlink-install
```

---

## 6. 键盘控制问题

### 6.1 按键无响应

**可能原因**：
- 终端未正确捕获键盘输入
- 键盘控制节点未启动

**解决方案**：
- 确保在正确的终端窗口中按键
- 检查节点是否正常运行：`ros2 node list | grep keyboard`

### 6.2 机器人不运动

**检查步骤**：
```bash
# 1. 检查控制模式
# 确保已按 '2' 或 '4' 切换到站立或小跑模式

# 2. 检查话题
ros2 topic echo /robot_mode
ros2 topic echo /cmd_vel

# 3. 检查控制器状态
ros2 lifecycle list
```

---

## 7. 联系支持

如果以上方法无法解决问题：
1. 查看 GitHub Issues
2. 提供详细的错误日志
3. 说明系统环境和复现步骤

---

## 3. 传感器配置问题

### 3.1 相机视野远处的物体（如墙壁）在图像和点云中消失

**问题现象**：
在 Gazebo 的主窗口（User Camera）中可以正常看到远处的墙壁和物体，但是机器人身上搭载的相机传感器（Sensor Camera）输出的图像和点云中，距离较远的物体却“消失”了。

**原因**：
机器人模型（URDF 或 SDF 文件）中，相机传感器的 `<far>`（远裁剪面）参数被设置得太小（例如 `10.0` 米）。当物体的距离超过了这个设定的数值时，相机插件就会停止渲染该物体，从而导致它在图像和点云中“消失”。

**解决方案**：
修改相机和深度相机的 `<far>` 裁剪面参数，将其放大（例如修改为 `300.0` 米）。
涉及的文件通常包括对应的 xacro 和 sdf 模型文件：
- `lite3_description/xacro/sensors/d435i_sensor.xacro`
- `quadruped_playground/models/D435/model.xacro`
- `quadruped_playground/models/D435/model.sdf`

修改后需要重新编译并使其生效：
```bash
colcon build --packages-select lite3_description quadruped_playground --symlink-install
```
