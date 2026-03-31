# FAST-LIVO2 到 Nav2 最小闭环验证记录（2026-03-24）

本文档记录 2026 年 3 月 24 日在 Lite3 Gazebo Classic 仿真环境中完成的一次实际导航联调过程与验证结论。

目标不是验证“所有导航能力都已完成”，而是验证以下最小闭环是否真实跑通：

`FAST-LIVO2 -> fastlivo_nav_bridge -> floor_mapper -> Nav2 -> /cmd_vel_nav -> /cmd_vel -> robot motion`

## 一、验证前提

在当前四足控制器中：

- `1` = `PASSIVE`
- `2` = `FIXEDDOWN`
- `3` = `FIXEDSTAND`
- `4` = `TROTTING`

实际联调中发现：

- 机器人需要先通过 `mode = 2` 停稳，避免 Gazebo 初始滑移污染 FAST-LIVO2 初始状态
- 机器人不能长时间停留在 `mode = 4` 而不发运动目标，否则容易失衡

因此，实际验证时采用如下策略：

- 先 `2 -> 3`
- 在 `mode = 3` 下完成 FAST-LIVO2、地图和 Nav2 状态检查
- 最后短时间切换到 `mode = 4`
- 切到 `mode = 4` 后立刻发送导航目标

## 二、实际验证流程

### 终端 1：启动仿真

```bash
cd /home/longkang/quadruped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash /home/longkang/quadruped_ws/src/lite3_gazebo_classic/run_gazebo.sh
```

### 终端 2：让机器人进入稳定准备态

```bash
cd /home/longkang/quadruped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /robot_mode std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
ros2 topic pub --once /robot_mode std_msgs/msg/Int32 "{data: 3}"
```

### 终端 3：启动 FAST-LIVO2

```bash
cd /home/longkang/quadruped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_livo mapping_gazebo.launch.py use_rviz:=false
```

### 终端 4：启动完整导航栈

```bash
cd /home/longkang/quadruped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch quadruped_nav_bringup navigation_main.launch.py
```

### 终端 5：检查感知与 Nav2 状态

```bash
cd /home/longkang/quadruped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /behavior_server

ros2 topic echo /aft_mapped_to_init --once
ros2 topic echo /cloud_registered --once
ros2 topic echo /odom --once
ros2 topic echo /floor_map --once
```

### 终端 6：观察 Nav2 输出

```bash
cd /home/longkang/quadruped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /cmd_vel_nav
```

### 终端 7：观察底层控制输出

```bash
cd /home/longkang/quadruped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /cmd_vel
```

### 终端 8：切换到 `mode = 4` 并立刻发送目标

```bash
cd /home/longkang/quadruped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /robot_mode std_msgs/msg/Int32 "{data: 4}" && sleep 0.5 && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{"pose":{"header":{"frame_id":"map"},"pose":{"position":{"x":0.3,"y":0.0,"z":0.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}}}'
```

### 紧急停止

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub --once /robot_mode std_msgs/msg/Int32 "{data: 3}"
```

## 三、实际观察结果

### 1. FAST-LIVO2 输出正常

实际观测到：

- `/aft_mapped_to_init` 正常发布
- `/cloud_registered` 正常发布

其中 `/aft_mapped_to_init` 的原始语义为：

- `header.frame_id = camera_init`
- `child_frame_id = aft_mapped`

这说明 FAST-LIVO2 上游工作正常。

### 2. bridge 正常

实际观测到：

- `/odom` 正常发布
- `header.frame_id = odom`
- `child_frame_id = base`

这说明 `fastlivo_nav_bridge` 已经把 FAST-LIVO2 输出整理成导航层可消费的标准 odom 接口。

### 3. `floor_mapper` 正常

实际观测到：

- `/floor_map` 正常发布 `nav_msgs/msg/OccupancyGrid`
- 地图 `frame_id = map`
- 分辨率、宽高和 origin 均正常

这说明 `floor_mapper` 已经不再是骨架，而是实际参与了导航闭环。

### 4. Nav2 正常

以下生命周期节点都达到 `active [3]`：

- `/planner_server`
- `/controller_server`
- `/bt_navigator`
- `/behavior_server`

这说明完整 Nav2 bringup 已经正常进入工作状态。

### 5. Action 正常

实际观测到：

- `/navigate_to_pose` goal 被 action server 接受
- 返回 `Goal accepted`

这说明上层导航目标入口已经正常接通。

### 6. 控制输出链正常

实际观测到：

- `/cmd_vel_nav` 有实际输出
- 输出中既出现过前进线速度，也出现过角速度修正
- `/cmd_vel` 与 `/cmd_vel_nav` 对应

这说明：

- Nav2 控制器在工作
- 速度已经从导航层传到四足控制主接口

### 7. 机器人实际运动正常

在一次近距离目标测试中：

- 目标设置为 `x = 0.3, y = 0.0`
- 机器人实际发生前进
- action 结果返回 `SUCCEEDED`

这说明最小自主导航闭环已经至少成功执行过一次。

## 四、验证结论

截至 2026 年 3 月 24 日，已经实际验证通过以下完整链路：

`FAST-LIVO2 -> fastlivo_nav_bridge -> floor_mapper -> Nav2 -> /cmd_vel_nav -> /cmd_vel -> robot motion`

因此，当前项目状态已经不再是“只有接口骨架”，而是已经完成一次真实可运行的最小导航闭环验证。

## 五、当前仍需继续验证的内容

虽然闭环已经跑通，但当前还未完全验证以下内容：

- 更远距离目标下的稳定性
- 多次重复目标下的一致性
- 长时间运行时 `map/odom/base` 语义是否持续稳定
- `mode = 4` 下的长期稳定性
- 目标容差是否过大导致过早判定 `SUCCEEDED`

因此，当前阶段结论应表述为：

- 已跑通一次最小闭环
- 进入稳定性与重复性优化阶段
