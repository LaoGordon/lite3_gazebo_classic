# PCD 到 2D 静态地图验证记录（2026-03-31）

本文档记录 2026 年 3 月 31 日在 Lite3 Gazebo Classic 工作区中完成的一次离线静态地图生成验证。

本次验证的目标不是正式接入 `map_server`，而是先确认：

- FAST-LIVO2 原生导出的全局 PCD 能否被稳定转换为 2D 静态地图
- 第一版离线投影流程是否可用
- 输出结果在人工观察下是否具备作为静态导航地图原料的潜力

## 一、输入数据

本次使用的输入点云为 FAST-LIVO2 原生落盘得到的下采样全局点云：

- `src/lite3_gazebo_classic/src/FAST-LIVO2/Log/PCD/all_downsampled_points.pcd`

该点云来自：

- `ros2 launch fast_livo mapping_gazebo.launch.py enable_pcd_save:=true`

FAST-LIVO2 退出时的实际输出记录为：

- 原始点云：928443 points
- 下采样点云：14927 points

## 二、使用的离线脚本

本次使用了新增的离线转换脚本：

- `navigation/quadruped_nav_bringup/scripts/pcd_to_static_map.py`

该脚本当前完成的功能包括：

- 读取 PCD 文件
- 提取 `x / y / z` 坐标
- 按高度范围裁剪
- 投影到 2D 平面
- 输出 `.pgm + .yaml`
- 额外输出一份参数元数据文件

## 三、本次使用参数

本次生成静态地图时使用的参数为：

- `z_min = -0.20`
- `z_max = 0.60`
- `resolution = 0.05`
- `padding = 1.0`
- `min_points_per_cell = 1`
- `frame_id = camera_init`

## 四、生成结果

本次生成的输出文件为：

- `navigation/quadruped_nav_bringup/maps/floor_1.pgm`
- `navigation/quadruped_nav_bringup/maps/floor_1.yaml`
- `navigation/quadruped_nav_bringup/maps/floor_1_metadata.txt`

脚本输出摘要如下：

- Map size: `420 x 468 cells`
- Occupied cells: `790`
- Origin: `(-9.900, -11.850)`
- Resolution: `0.05 m`

## 五、人工观察结论

对生成出的 `floor_1.pgm` 进行人工查看后，当前结论为：

- 第一版地图生成流程已经跑通
- 输出图在视觉上效果较好，整体轮廓已经具备静态平面图的基本形态
- 从当前人工检查结果看，这张图已经具备继续作为 `map_server` 输入原料推进的价值
- 说明当前导出的全局 PCD 具备继续转为静态导航地图的基础

这意味着：

- 当前 `PCD -> 2D map` 路线已经从“可尝试”进入“可继续工程化推进”
- 后续工作不再是“能不能生成”，而是“如何继续收敛参数和接入导航栈”

## 六、当前边界

虽然第一版结果已经可用，但当前仍有几个边界需要明确：

- 当前结果仍属于第一版离线投影结果
- 尚未验证其在 `map_server` + `global_costmap` 下的实际效果
- 高度裁剪范围和栅格参数后续仍可能继续调整
- 当前地图语义仍应理解为来自 `camera_init` 参考系下的静态原料，而不是已经正式闭合到最终 `/map`

## 七、下一步建议

基于本次结果，下一步建议为：

1. 继续人工检查 `floor_1.pgm` 的结构是否与场景一致
2. 根据效果微调 `z_min / z_max / resolution / padding`
3. 在 `quadruped_nav_bringup` 中引入 `map_server` 的最小加载链路
4. 用该静态地图替换当前过渡性的 `global_costmap` 配置
5. 再进一步收敛 `map -> odom` 的正式语义

## 八、当前阶段结论

截至 2026 年 3 月 31 日，可以认为以下结论已经成立：

- FAST-LIVO2 原生导出的全局 PCD 可被离线转换为 2D 静态地图
- 第一版 `floor_1` 已经成功生成
- 人工观察结果表明输出效果具备继续推进的价值
- 当前项目已经从“全局点云落盘”进入“静态地图接入 Nav2”的下一阶段


## 九、补充验证：`map_server` 与 RViz 加载（2026-04-01）

在第一版静态地图文件生成完成后，又完成了 `map_server` 加载与 RViz 显示验证。

### 1. `map_server` 验证结果

已确认：

- `ros2 lifecycle get /map_server` 返回 `active [3]`
- `ros2 topic echo /map --once` 能正确收到 `nav_msgs/OccupancyGrid`
- 实际加载地图文件为安装目录下的：
  - `install/quadruped_nav_bringup/share/quadruped_nav_bringup/maps/floor_1.pgm`

从实际回显中可以确认：

- `frame_id = map`
- `resolution = 0.05`
- `width = 420`
- `height = 468`
- `origin = (-9.9, -11.85)`

这说明当前链路已经进一步成立：

`floor_1.pgm/.yaml -> map_server -> /map`

### 2. RViz 调试记录

初次在 RViz 中添加 `Map` display 时，出现了 `No map received`。

后续确认该问题不是 `map_server` 没有发布地图，而是 RViz 订阅 `/map` 时的 QoS 设置不匹配。

有效的修正方式为：

- `Fixed Frame` 设为 `map`
- `Map` display 的 `Topic` 设为 `/map`
- `Durability Policy` 设为 `Transient Local`

在此基础上，若 TF 树中暂时没有 `map` frame，还需要额外提供临时 TF，例如：

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

修正后，RViz 已能正常显示静态地图。

### 2.1 补充问题：RViz 显示 `No map received`，但 `/map` 实际已发布

后续复现中又遇到一种更隐蔽的情况：

- `ros2 lifecycle get /map_server` 返回 `active [3]`
- `ros2 topic echo /map --once` 能正常收到 `nav_msgs/OccupancyGrid`
- RViz 的 `Map` display 仍然显示 `No map received`

这说明问题不在 `map_server`，而在 RViz 对 `/map` 的实际订阅 QoS。

可使用以下命令确认：

```bash
ros2 topic info /map -v
```

若输出表现为：

- `map_server` 发布者：`Durability = TRANSIENT_LOCAL`
- `rviz` 订阅者：`Durability = VOLATILE`

则可确认根因是：

- RViz 实际仍在用 `VOLATILE` 订阅 `/map`
- 而 `/map` 是以 `TRANSIENT_LOCAL` 方式发布的
- 两侧 Durability 不匹配，因此 RViz 即使界面中看起来配置正确，也仍会收不到地图

本次复现中，最稳定的解决方式不是只修改已有 `Map` display 的参数，而是：

1. 删除当前已有的 `Map` display
2. 重新启动 `rviz2`
3. 重新添加一个新的 `Map` display
4. 显式设置：
   - `Reliability Policy = Reliable`
   - `Durability Policy = Transient Local`
   - `History Policy = Keep Last`
   - `Depth = 1`
   - `Topic = /map`

经验结论：

- 仅在已有 `Map` display 上修改 QoS，RViz 不一定会真正重建订阅
- 因此会出现“界面看起来已经改成 `Transient Local`，但 `ros2 topic info /map -v` 中 RViz 实际仍为 `VOLATILE`”的现象
- 遇到这种情况时，应优先采用“删除 display 或重启 RViz 后重新添加”的方式处理

### 3. 当前阶段结论更新

截至 2026-04-01，可以进一步确认：

- 第一版静态地图文件不仅已经成功生成，而且已经能被 `map_server` 正常加载
- `/map` 话题已经可以稳定发布
- RViz 侧的关键注意事项是：静态地图显示需要将 `Durability Policy` 设置为 `Transient Local`
- 当前项目已经完成从“离线做图”到“静态地图上线发布”的验证闭环

### 4. 下一步影响

这次补充验证完成后，下一步就不再是“地图能不能发布”，而是：

1. 将 `global_costmap` 从当前过渡配置切回标准 `StaticLayer -> /map`
2. 验证全局规划器是否真正开始使用这张静态地图
3. 最后再继续收敛 `map -> odom` 的正式语义


## 十、后续复现补充（2026-04-03）

### 1. 地图文件不要长期放在 `/tmp`

在后续验证中发现，将生成出的 `.pgm/.yaml` 长期放在 `/tmp` 下不够稳妥。

原因包括：

- 系统重启后 `/tmp` 中的文件可能被清理
- 重新打开 `map_server` 时，若 `map_yaml` 指向 `/tmp` 中已不存在的文件，会直接报错
- 不利于后续将同一张地图反复用于 `map_server`、Nav2 与文档记录

因此，后续验证中应优先将地图保存到仓库内的统一目录：

- `navigation/quadruped_nav_bringup/maps/`

本次复现中，已将验证地图固化为：

- `navigation/quadruped_nav_bringup/maps/floor_small_test.pgm`
- `navigation/quadruped_nav_bringup/maps/floor_small_test.yaml`
- `navigation/quadruped_nav_bringup/maps/floor_small_test_metadata.txt`

后续建议统一使用该仓库内路径，而不是继续依赖 `/tmp/floor_small_test.yaml`。

### 2. 一键验证脚本

为了避免每次验证时分别手动启动 Gazebo、FAST-LIVO2、bridge、静态地图与导航栈，当前已新增一个总入口脚本：

- `run_navigation_validation.sh`

之所以最终采用脚本而不是继续使用单个总 launch，原因是：

- Gazebo Classic 的稳定启动依赖仓库内已经验证可用的 `run_gazebo_world.sh`
- 该脚本中包含了残留进程清理、共享内存清理与 GUI 环境准备
- 导航链还需要按顺序分阶段启动，直接一次性拉起所有节点不够稳定

当前脚本会按顺序启动：

1. `run_gazebo_world.sh`
2. FAST-LIVO2
3. `fastlivo_nav_bridge`
4. `static_map.launch.py`
5. 临时 `map -> odom` 静态 TF
6. `navigation_main.launch.py`
7. 导航 RViz

脚本内部在各阶段之间加入了固定延时，以等待 Gazebo、FAST-LIVO2 与 TF 主干逐步稳定。

默认地图文件使用：

- `navigation/quadruped_nav_bringup/maps/floor_small_test.yaml`

使用方式：

```bash
cd ~/ws/quadruped_ws/src/lite3_gazebo_classic
./run_navigation_validation.sh
```

若要显式指定 world，可将 world 文件路径作为第一个参数传入：

```bash
cd ~/ws/quadruped_ws/src/lite3_gazebo_classic
./run_navigation_validation.sh /absolute/path/to/your.world
```

若要调整启动节奏，可通过环境变量修改延时，例如：

```bash
cd ~/ws/quadruped_ws/src/lite3_gazebo_classic
GAZEBO_DELAY=12 FASTLIVO_DELAY=8 BRIDGE_DELAY=5 STATIC_MAP_DELAY=4 NAV2_DELAY=5 ./run_navigation_validation.sh
```

### 3. 关于 `map -> odom` 临时 TF

是的，当前一键脚本中已经内置了你之前手动执行的这条命令：

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

当前用途是：

- 在验证阶段先把 `map -> odom` 以恒等变换接起来
- 让 `map -> odom -> base` 这条 TF 主干可用于 RViz 和 Nav2 验证

但这仍然只是验证阶段的临时方案，不应视为最终定位语义。

### 4. 当前阶段建议

截至 2026-04-03，后续验证时更推荐采用以下流程：

1. 将静态地图保存在 `navigation/quadruped_nav_bringup/maps/`
2. 通过 `run_navigation_validation.sh` 一键拉起验证链
3. 在 RViz 中优先关注：
   - `/global_costmap/costmap`
   - 路径规划结果
   - 机器人是否能沿路径正常运动
4. 若机器人倒地或状态明显异常，直接整套重启验证链，而不是在污染状态下继续测试
