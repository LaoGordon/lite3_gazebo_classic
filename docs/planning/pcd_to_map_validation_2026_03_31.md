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
