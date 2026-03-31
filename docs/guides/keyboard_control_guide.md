# 键盘控制使用指南

本文档详细描述了如何使用键盘控制 Lite3 四足机器人。

## 目录

1. [快速开始](#1-快速开始)
2. [控制模式](#2-控制模式)
3. [按键映射](#3-按键映射)
4. [使用示例](#4-使用示例)
5. [常见问题](#5-常见问题)

---

## 1. 快速开始

### 1.1 编译

```bash
cd ~/quadruped_ws
colcon build --packages-select keyboard_input
```

### 1.2 启动

```bash
# 终端 1：启动 Gazebo 仿真
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py

# 终端 2：启动键盘控制节点
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

### 1.3 前提条件

- 确保 `keyboard_input` 节点和 `unitree_guide_controller` 都已启动
- 键盘控制通过 `/cmd_vel` 和 `/robot_mode` 话题发送控制指令

---

## 2. 控制模式

键盘数字键 `1-4` 用于切换主控制链模式：

| 按键 | 模式代码 | 模式名称 | 说明 |
|------|----------|----------|------|
| `1` | 1 | Passive | 被动模式（Passive）- 电机断电 |
| `2` | 2 | FixedDown | 收腿模式 |
| `3` | 3 | FixedStand | 站立模式 |
| `4` | 4 | Trotting | 小跑模式（Trot）- 对角线步态行走 |

### 模式切换流程

1. 启动后，机器人默认处于**被动模式**（模式 1）
2. 按 `2` 切换到**收腿模式**
3. 按 `3` 切换到**站立模式**，机器人会站起来
4. 按 `4` 切换到**小跑模式**，机器人可以行走
5. 按 `空格键` 重置所有速度指令

---

## 3. 按键映射

### 3.1 左摇杆（WASD）- 平移控制

| 按键 | 方向 | 说明 |
|------|------|------|
| `W` | 前进 | 机器人向前移动 |
| `S` | 后退 | 机器人向后移动 |
| `A` | 左移 | 机器人向左平移 |
| `D` | 右移 | 机器人向右平移 |

### 3.2 右摇杆（IJKL）- 旋转控制

| 按键 | 方向 | 说明 |
|------|------|------|
| `I` | 逆时针 | 机器人逆时针旋转 |
| `K` | 顺时针 | 机器人顺时针旋转 |
| `J` | 左转 | 机器人向左转头 |
| `L` | 右转 | 机器人向右转头 |

### 3.3 功能键

| 按键 | 功能 |
|------|------|
| `空格` | 重置速度指令（停止运动） |
| `Ctrl+C` | 退出键盘控制节点 |

---

## 4. 使用示例

### 4.1 启动机器人并行走

```bash
# 步骤 1：启动仿真（终端 1）
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py

# 步骤 2：启动键盘控制（终端 2）
source install/setup.bash
ros2 run keyboard_input keyboard_input

# 步骤 3：控制机器人
# 按 '2' - 机器人站起来（固定站立模式）
# 按 '4' - 切换到小跑模式
# 按 'W' - 机器人前进
# 按 'S' - 机器人后退
# 按 'A/D' - 机器人左右平移
# 按 'Q/E' - 机器人旋转
# 按 '空格' - 停止运动
```

### 4.2 监控控制话题

```bash
# 在另一个终端查看控制指令
ros2 topic echo /robot_mode
ros2 topic echo /cmd_vel
```

### 4.3 速度控制说明

- `W/S`：控制 `/cmd_vel.linear.x`
- `A/D`：控制 `/cmd_vel.linear.y`
- `Q/E`：控制 `/cmd_vel.angular.z`
- 数字键 `1-4`：发布 `/robot_mode`

默认速度范围：
- `linear.x`: `[-0.4, 0.4]`
- `linear.y`: `[-0.3, 0.3]`
- `angular.z`: `[-0.2, 0.2]`

---

## 5. 常见问题

### 5.1 按键无响应

**可能原因**：
- 终端未正确捕获键盘输入
- 键盘控制节点未启动

**解决方案**：
- 确保在正确的终端窗口中按键
- 检查节点是否正常运行：`ros2 node list | grep keyboard`

### 5.2 机器人不运动

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

### 5.3 机器人持续保持上一次速度

当前键盘控制是**增量式速度设定**：
- 按一次运动键会调整当前速度目标
- 按 `空格` 会将速度清零
- 如需验证当前输出，请查看 `/cmd_vel`

### 5.4 模式切换失败

**可能原因**：
- 控制器未正确加载
- 状态机未初始化完成

**解决方案**：
- 等待控制器完全启动后再切换模式
- 检查控制器日志是否有错误

---

## 6. 节点参数

### 6.1 发布的话题

| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/robot_mode` | std_msgs/msg/Int32 | 主链模式切换 |
| `/cmd_vel` | geometry_msgs/msg/Twist | 速度控制指令 |

---

## 7. 高级用法

### 7.1 自定义速度步长

可通过节点参数调整：

```bash
ros2 run keyboard_input keyboard_input --ros-args \
  -p linear_step:=0.05 \
  -p lateral_step:=0.05 \
  -p angular_step:=0.05
```

### 7.2 与其他控制方式共存

键盘控制可以与以下控制方式共存：
- 手柄控制（joystick_input）
- Unitree 手柄（unitree_joystick_input）

**注意**：多个控制源同时发送指令可能导致冲突。

---

## 8. 参考文档

- [传感器配置](./sensor_configuration.md)
- [控制接口规范](../references/control_interface_spec.md)
