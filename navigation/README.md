# 导航集成层

该目录用于承载 Lite3 仿真工作区中的导航集成能力。

这里的目标不是替换现有的四足控制器，也不是替换 FAST-LIVO2 本身，而是在当前仿真系统之上建立一层清晰的“导航适配层”，把已有的感知、控制、仿真资源整理成后续可持续扩展的导航系统。

这层能力后续将支撑：

- 基于 Nav2 的单层导航
- 楼梯区域的特殊通行模式切换
- 多楼层任务执行
- 面向机器狗的局部 3D 可通行性导航扩展

当前阶段，这个目录主要完成第一件必须先做对的事情：

- 将 FAST-LIVO2 的输出整理为导航友好的 ROS 2 标准接口
- 为后续“点云转导航地图/障碍输入”预留独立模块位置
- 提供导航层统一的 bringup 入口

## 为什么需要这一层

当前仓库已经具备三类核心能力：

- 四足机器人仿真与控制链路
- FAST-LIVO2 激光雷达/IMU 里程计与建图能力
- 对外标准运动接口，例如 `/cmd_vel` 与 `/robot_mode`

但是，这些能力还没有被组织成一个标准导航系统。

FAST-LIVO2 当前输出的是更偏算法内部风格的话题和坐标语义，而 Nav2、RViz、costmap 以及后续导航模块更希望使用标准化输入，例如：

- `/odom`
- `map -> odom -> base` 的 TF 树
- 统一、稳定的 frame 命名
- 具有明确参考系的障碍点云或扫描输入

如果没有这一层适配，后续导航模块就必须直接依赖 FAST-LIVO2 当前的话题命名、坐标语义和仿真接线方式。这样做的问题很明显：

- 调试困难
- 模块边界混乱
- 后续引入楼梯、多楼层、局部 3D 规划时耦合会越来越重

因此，这个 `navigation/` 目录的本质角色是：

- 向上承接感知与仿真
- 向下提供标准化导航接口

它是“上游感知”和“下游导航”之间的集成边界。

## 设计原则

本目录下的导航集成层遵循以下原则：

- 低层四足控制器与导航层分离
- FAST-LIVO2 继续作为主感知与主定位来源
- 将算法风格输出转换为标准 ROS 导航接口
- 通过多个独立 package 演进，而不是把所有逻辑堆进一个大包
- 每一阶段先单独验证，再进入下一阶段

这种拆分是刻意为之。

四足控制器应继续只负责运动执行；导航层应负责感知接口整理、地图转换、任务组织与导航逻辑编排。这样后面无论替换 Nav2、增加楼层管理器，还是引入楼梯局部规划器，都不会破坏底层控制结构。

## 目录结构

当前第一阶段 package：

- `fastlivo_nav_bridge`
  - 将 FAST-LIVO2 的里程计与点云输出转换为导航侧标准话题和 TF
- `floor_mapper`
  - 预留用于把 3D 点云转换为单层 2D 导航输入
- `quadruped_nav_bringup`
  - 导航层的统一启动入口与配置汇总位置

计划中的第二阶段 package：

- `multifloor_nav_manager`
  - 管理楼层切换、楼层地图与跨楼层任务流转
- `stair_detector`
  - 检测楼梯或通行性敏感区域
- `stair_local_planner`
  - 负责楼梯区域或约束环境下的局部通行策略

## 当前阶段总体架构

当前第一阶段的数据流可以概括为：

`FAST-LIVO2 -> fastlivo_nav_bridge -> /odom + TF + /obstacle_points -> floor_mapper -> 后续 Nav2 输入`

每一层的作用如下：

- FAST-LIVO2 负责估计机器人位姿并发布配准后的点云
- `fastlivo_nav_bridge` 负责把这些输出转换为标准导航接口
- `floor_mapper` 后续负责把 3D 感知结果压缩成单层导航可用的 2D 输入
- `quadruped_nav_bringup` 负责统一拉起这些模块

因此，第一阶段的重点不是“完整导航”，而是“接口标准化”。

## 各 package 的职责说明

### `fastlivo_nav_bridge`

这是当前第一阶段中已经具备实际功能的核心 package。

当前职责：

- 订阅 FAST-LIVO2 的 `/aft_mapped_to_init`
- 订阅 FAST-LIVO2 的 `/cloud_registered`
- 发布标准 `/odom`
- 发布 `map -> odom`
- 发布 `odom -> base`
- 将点云重发为 `/obstacle_points`

它的实现原理很直接：

- FAST-LIVO2 已经完成了位姿估计
- bridge 不重复做定位，也不参与建图求解
- 它只负责把 FAST-LIVO2 的结果整理为标准 ROS 导航接口

当前 frame 约定为：

- `map`：全局导航参考系
- `odom`：局部连续参考系
- `base`：导航层使用的机器人本体参考系

当前第一版有一个刻意保留的简化：

- `map -> odom` 先发布为恒等变换

这样做的原因是：现阶段最重要的是先把接口链打通，让 RViz、Nav2 和后续模块可以消费这套数据，而不是一开始就处理回环修正、全局重定位等更复杂问题。后面如果需要处理回环一致性，再进一步增强这一层即可。

**Frame 语义风险说明**：

当前 `fastlivo_nav_bridge` 对 `/obstacle_points` 的处理方式是：

- 直接复制 `/cloud_registered` 的点云数据
- 将 `header.frame_id` 设置为 `map`
- 发布为 `/obstacle_points`

这一做法成立的前提是：

- `/cloud_registered` 本身已经在全局语义下表达
- 或者 `/cloud_registered` 和 `map` 在当前链路里可视为一致

**风险边界**：

- 当前做法**不是**"已经严格完成点云 TF 变换"
- 而是"当前版本按现有 FAST-LIVO2 语义整理为 map 侧障碍输入"
- 若 `/cloud_registered` 原始 frame 语义与 `map` 不完全一致，地图可能整体偏移

**后续验证项**：

- 核对 `/cloud_registered` 原始 frame 语义
- 核对 `/obstacle_points` 与 `/floor_map`、机器人位姿的一致性
- 必要时改成显式 TF transform，而不是只改 header

### `floor_mapper`

这个 package 当前已实现第一版 `OccupancyGrid` 投影功能。

职责是：

- 订阅 `/obstacle_points`
- 对 3D 点云进行过滤、裁剪、投影
- 生成单层导航可用的 2D 输入（`OccupancyGrid`）

已实现的处理步骤包括：

- 高度裁剪：限制只保留当前楼层导航真正关心的障碍高度范围
- 距离裁剪：限制地图更新范围，避免过远点云影响局部导航输入稳定性
- 自体剔除：避免机器人机身、腿部或近体噪声被误判为障碍
- 2D 投影：把 3D 感知结果压缩成单层导航可直接消费的平面表示

之所以需要这个包，是因为整个系统长期目标不是纯 2D 导航，但短期最稳妥、工程成本最低的方式仍然是：

- 感知保持 3D
- 平坦楼层区域先抽象成 2D 导航问题

这样既保留了 FAST-LIVO2 的意义，也便于逐步接入 Nav2。

### `quadruped_nav_bringup`

这个 package 负责导航层的统一启动。

职责包括：

- 统一拉起 bridge 和后续 mapping 模块
- 汇总导航层参数与 launch 入口
- 作为未来整个导航栈的统一启动点

这个包应当保持“编排层”角色，不应承载核心算法逻辑。核心功能应继续放在各自独立 package 中。

## 第一步工作的作用

当前整个项目的第一步，不是做路径规划，而是先把接口做正确。

在接入 Nav2、楼梯模式、多楼层管理之前，系统必须先稳定回答这几个基本问题：

- 机器人当前在哪里
- 这个位姿是在哪个参考系下表达的
- 下游模块应该从哪里读取这个位姿
- 障碍物点云是在哪个参考系下发布的
- TF 树是否满足标准 ROS 导航工具的预期

这就是为什么第一阶段先做 `fastlivo_nav_bridge`。

这一步的价值在于，它为后续所有模块建立了一份稳定的“接口契约”。有了这层契约后，后续模块就不再需要直接依赖 FAST-LIVO2 当前内部风格的话题命名和 frame 语义。

## 第一步的实现原理

第一步的核心原理可以分成三部分。

### 1. 位姿标准化

FAST-LIVO2 会发布自己的里程计结果，例如 `/aft_mapped_to_init`。

这个话题本质上表达的是：

- 当前机器人位姿
- 该位姿在 FAST-LIVO2 所使用的参考系下的估计结果

bridge 所做的事情，是把它整理成标准 `nav_msgs/msg/Odometry`：

- `header.frame_id = odom`
- `child_frame_id = base`
- `pose.pose = 当前机器人姿态`

这样后续标准导航模块就可以直接消费 `/odom`。

### 2. TF 树标准化

ROS 导航系统实际依赖的不仅仅是 `/odom`，还依赖标准 TF 树。

当前桥接节点发布：

- `map -> odom`
- `odom -> base`

从而形成标准导航链：

`map -> odom -> base`

这一点的意义是：

- RViz 可以正确显示机器人位置
- Nav2 可以获取机器人位姿
- 后续传感器或障碍数据可以被正确转换到目标参考系中

当前把 `map -> odom` 先做成恒等变换，是为了优先解决“能接上标准工具链”的问题，而不是一开始就引入更复杂的全局修正逻辑。

### 3. 感知输入标准化

FAST-LIVO2 还会输出配准后的点云，例如 `/cloud_registered`。

这些点云虽然对算法本身有用，但对下游导航而言，还需要满足：

- 具有稳定、明确的 frame
- 通过统一话题提供给地图模块和 costmap 模块

所以 bridge 当前会先将其重发为 `/obstacle_points`，作为后续 `floor_mapper` 和其他导航模块的标准输入源。

## 为什么验证时必须先发 `robot_mode = 2`

这一点在当前仿真里非常重要。

实际验证中已经发现：如果机器人在 Gazebo 刚启动时不先停稳，它会以非常缓慢的方式滑动。这个滑动虽然看起来很小，但足以影响 FAST-LIVO2 的初始状态，使上游 odom 出现异常漂移或不可信结果。

这会导致一个很危险的误判：

- 看上去是 bridge 输出错了
- 实际上是 FAST-LIVO2 上游输入状态已经被仿真初始滑移污染了

因此，当前阶段推荐的验证前置条件是：

- 先发布 `/robot_mode = 2`
- 再发布零速度 `/cmd_vel`
- 等机器人稳定
- 然后再启动 FAST-LIVO2 和导航 bridge

这样做的意义是控制变量，把“仿真接触初始不稳定”从“导航接口是否正确”这个问题里剥离出去。否则你验证到的就不是 bridge 本身，而是多个问题叠加后的结果。

## 当前已经验证了什么

截至 2026 年 3 月 24 日，在 Gazebo 仿真环境中已经完成两轮验证：

- 第一轮：接口标准化验证
- 第二轮：最小 Nav2 闭环验证

第一轮验证中，在“先发 `/robot_mode = 2` 让机器人停稳”的前提下，以下内容已经重复验证通过。

当前已确认：

- FAST-LIVO2 上游里程计话题存在
- FAST-LIVO2 注册点云话题存在
- `fastlivo_nav_bridge` 可以正确订阅这些输入
- `/odom` 可以正常发布
- `/obstacle_points` 可以正常发布
- `map -> odom -> base` TF 链可以正常建立

第二轮验证中，已经进一步确认：

- `floor_mapper` 可以将 `/obstacle_points` 转换为 `/floor_map`
- `quadruped_nav_bringup` 可以正常拉起 Nav2
- `planner_server`、`controller_server`、`bt_navigator`、`behavior_server` 可以进入 `active`
- `/navigate_to_pose` 目标可以被正常接受
- Nav2 可以输出 `/cmd_vel_nav`
- 底层主控制链可以继续输出 `/cmd_vel`
- 机器人在一次近距离目标测试中已实际前进，action 结果返回 `SUCCEEDED`

这说明当前系统已经不只是“接口打通”，而是已经完成一次可运行的最小导航闭环：

`FAST-LIVO2 -> fastlivo_nav_bridge -> floor_mapper -> Nav2 -> /cmd_vel_nav -> /cmd_vel -> robot motion`

## 当前已实现内容与未实现内容

当前已实现：

- 导航层目录结构
- `fastlivo_nav_bridge` 第一版可工作实现
- `quadruped_nav_bringup` 统一启动入口
- `floor_mapper` 第一版 `OccupancyGrid` 投影实现
- Nav2 最小闭环接入与一次成功验收

当前未实现：

- 楼梯检测
- 楼层切换逻辑
- 局部 3D 通行规划

这里刻意将“已实现”和“计划实现”分开说明，是为了避免文档与实际代码状态不一致。

## 构建方式

在工作区根目录执行：

```bash
cd /home/longkang/quadruped_ws
colcon build --packages-select fastlivo_nav_bridge floor_mapper quadruped_nav_bringup --symlink-install
```

## 启动方式

当前只启动导航前半链路：

```bash
cd /home/longkang/quadruped_ws
source install/setup.bash
ros2 launch quadruped_nav_bringup navigation_bringup.launch.py
```

当前完整导航启动方式：

```bash
cd /home/longkang/quadruped_ws
source install/setup.bash
ros2 launch quadruped_nav_bringup navigation_main.launch.py
```

如果只想验证 bridge 和 `floor_mapper`，可以使用：

```bash
cd /home/longkang/quadruped_ws
source install/setup.bash
ros2 launch quadruped_nav_bringup navigation_main.launch.py enable_nav2:=false
```

注意：这些步骤默认仿真和 FAST-LIVO2 已经先启动。

## 当前推荐验证顺序

截至 2026 年 3 月 24 日，当前推荐的最小闭环验证顺序如下。

1. 使用 [run_gazebo.sh](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/run_gazebo.sh) 启动仿真
2. 发布 `/robot_mode = 2`
3. 发布零速度 `/cmd_vel`
4. 等待机器人稳定后切换到 `/robot_mode = 3`
5. 启动 FAST-LIVO2
6. 启动完整导航栈 `quadruped_nav_bringup navigation_main.launch.py`
7. 在机器人仍保持 `mode = 3` 时，检查：
   - `/aft_mapped_to_init`
   - `/cloud_registered`
   - `/odom`
   - `/obstacle_points`
   - `/floor_map`
   - `map -> odom -> base`
   - Nav2 核心 lifecycle 节点是否处于 `active`
8. 在目标和观察终端都准备好后，再快速切换到 `/robot_mode = 4`
9. 切换到 `mode = 4` 后立刻发送近距离导航目标，例如 `x = 0.3`
10. 观察：
   - `/cmd_vel_nav`
   - `/cmd_vel`
   - Gazebo 中机器人是否先转向再前进

这里有一个非常重要的控制约束：

- 不要让机器人长时间停留在 `mode = 4` 而不发运动目标

当前控制器在 `TROTTING` 模式下长时间原地保持并不稳定，因此实际联调时必须采用：

- `2 -> 3` 完成准备
- 先检查感知与导航链路
- 最后再短时间切换 `4` 并立刻发 goal

这个顺序的意义在于：

- 先稳定仿真
- 再确认上游定位与地图输入
- 最后验证完整导航闭环

这样每一层问题都能被清楚定位，同时可以避免机器人在 `mode = 4` 下空站过久导致失衡。

更具体的联调命令、终端分工和 2026-03-24 的实际验证结果，见：

- [navigation_validation_2026_03_24.md](/home/longkang/quadruped_ws/src/lite3_gazebo_classic/docs/navigation_validation_2026_03_24.md)

## 下一步工作

下一步不再是"是否能接上 Nav2"，而是进入稳定性与重复性验证阶段，重点包括：

- 更远目标与多次重复目标下的稳定性
- `mode = 4` 下的运行时稳定性
- 目标容差是否过大导致过早 `SUCCEEDED`
- 地图与 frame 语义在长时间运行下是否持续稳定

## 当前已完成内容

截至 2026 年 3 月 25 日，以下内容已经完成并验证：

### 已完成：接口标准化

- `fastlivo_nav_bridge` 已实现并发布：
  - `/odom`（标准里程计）
  - `map -> odom -> base` TF 链
  - `/obstacle_points`（障碍点云）

### 已完成：floor_mapper 第一版

- 已实现完整的点云到 2D 地图处理链：
  - 订阅 `/obstacle_points`
  - 高度裁剪
  - 距离裁剪
  - 自体剔除
  - 2D 栅格投影
  - 发布 `/floor_map`（`OccupancyGrid`）

### 已完成：Nav2 最小闭环接入

- `quadruped_nav_bringup` 已能拉起完整 Nav2
- 已验证核心 lifecycle 节点进入 `active`
- 已验证 `/navigate_to_pose` 目标可被接受并返回 `SUCCEEDED`
- 已验证从目标到 `/cmd_vel` 的完整运动输出链

### 后续阶段任务

在最小闭环已打通的基础上，后续计划包括：

### 第二阶段：稳定化与参数收敛

目标是让已跑通的闭环具备更好的重复性和稳定性。

计划内容：

1. 调整 `xy_goal_tolerance` 与 `yaw_goal_tolerance`
2. 收敛速度参数，避免过激控制
3. 降低偶发性和误判成功的概率
4. 建立标准测试流程与回归脚本

### 第三阶段：为楼梯和多楼层做扩展准备

目标是建立"平地用 Nav2，楼梯区域切换特殊模式"的扩展路径。

计划内容：

1. 增加 `stair_detector`
   - 从局部点云或几何特征中识别楼梯/坡道区域
2. 增加 `multifloor_nav_manager`
   - 管理楼层切换点、楼层地图和跨楼层任务流
3. 增加 `stair_local_planner`
   - 处理楼梯区域的低速、受约束局部通行

这一阶段的交付结果应当是：

- 系统从"单层导航"扩展到"多楼层任务 + 楼梯特例处理"的架构雏形

### 当前开发策略

为了避免在系统尚未稳定时过早引入复杂性，当前建议始终遵循以下开发顺序：

1. 先把输入接口做标准化并验证通过（已完成）
2. 再把 3D 点云转换成可用的单层导航输入（已完成）
3. 再接 Nav2 跑通平地闭环（已完成）
4. 稳定最小闭环（当前优先级）
5. 最后再处理楼梯、楼层切换和局部 3D 通行
