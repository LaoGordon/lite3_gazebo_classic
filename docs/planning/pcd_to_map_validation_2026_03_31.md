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
