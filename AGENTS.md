# AGENTS.md

本文件面向首次进入本仓库的 AI 代理或新开发者，用于快速建立项目上下文，避免重复误判当前导航架构。

## 1. 项目概述

本仓库是 Lite3 四足机器人在 Gazebo Classic 下的 ROS 2 Humble 仿真环境。

当前主线工作不再是“打通最小闭环”，而是把已经可运行的导航链路从过渡态推进到真正的静态 `/map` 全局导航架构。

当前已跑通的基础链路为：

`FAST-LIVO2 -> fastlivo_nav_bridge -> floor_mapper -> Nav2 -> /cmd_vel -> robot motion`

## 2. 关键目录

- `navigation/`
  - 当前导航相关代码与 bringup
- `navigation/floor_mapper/`
  - 在线局部 2D 障碍投影模块
- `navigation/fastlivo_nav_bridge/`
  - FAST-LIVO2 到导航接口的桥接层
- `navigation/quadruped_nav_bringup/`
  - Nav2 参数、launch、地图资源与离线脚本
- `src/FAST-LIVO2/`
  - FAST-LIVO2 子模块
- `docs/`
  - 项目文档索引、规划文档、历史记录

## 3. 先读哪些文档

首次进入仓库时，建议优先阅读以下文件：

1. `docs/README.md`
2. `docs/planning/navigation_issues_2026_03_31.md`
3. `docs/planning/static_map_pipeline_design_2026_03_31.md`
4. `docs/planning/fastlivo2_pcd_export_guide_2026_03_31.md`
5. `docs/planning/pcd_to_map_validation_2026_03_31.md`

如果需要历史背景，再看：

- `docs/history/navigation_validation_2026_03_24.md`
- `docs/history/navigation_status_2026_03_25.md`
- `docs/history/navigation_progress_2026_03_30.md`

## 4. 当前架构状态

当前项目的关键状态如下：

- Nav2 最小闭环已经跑通
- `global_costmap` 已从误用 `/floor_map` 的旧配置中止血，目前处于过渡配置
- FAST-LIVO2 原生 PCD 落盘已经验证可用
- 第一版离线 `PCD -> 2D static map` 已成功生成 `floor_1.pgm/.yaml`
- 人工检查结果表明第一版静态图效果可接受，具备继续接入 `map_server` 的价值

## 5. 不要误解的点

### 5.1 `floor_mapper` 不是全局建图器

`floor_mapper` 当前职责是：

- 将近场 3D 点云投影为局部 2D 障碍图
- 服务于局部避障或局部代价图

它不是长期静态楼层地图，不应继续作为 `global_costmap` 的全局静态图来源。

### 5.2 `fastlivo_nav_bridge` 当前有 frame 语义债务

已确认：

- `/cloud_registered_lidar.header.frame_id = camera_init`
- `/aft_mapped_to_init.header.frame_id = camera_init`
- `/aft_mapped_to_init.child_frame_id = aft_mapped`

当前 bridge 能工作，是因为它通过 frame 重映射和近似恒等的 `map -> odom` 支撑最小闭环。

这不是最终语义。一旦真正引入静态 `/map`，`map -> odom` 需要重新定义。

### 5.3 当前静态地图仍处于离线验证阶段

`navigation/quadruped_nav_bringup/maps/floor_1.yaml` 与 `floor_1.pgm` 已生成，但尚未正式接入 `map_server` 与 `global_costmap` 主流程。

## 6. 当前主线任务

当前推荐按以下顺序推进：

1. 保持现有最小闭环继续可运行
2. 使用离线 PCD 继续收敛第一版 2D 静态地图参数
3. 在 `quadruped_nav_bringup` 中引入最小 `map_server` 链路
4. 将 `global_costmap` 从过渡配置切换到标准 `StaticLayer -> /map`
5. 最后再正式收敛 `map -> odom` 的语义和实现

## 7. 常用入口

### 7.1 仿真

- 根 README: `README.md`
- 文档索引: `docs/README.md`

### 7.2 FAST-LIVO2

常用启动命令：

```bash
ros2 launch fast_livo mapping_gazebo.launch.py
```

建图并导出全局 PCD：

```bash
ros2 launch fast_livo mapping_gazebo.launch.py use_rviz:=true enable_pcd_save:=true
```

### 7.3 离线静态地图生成

脚本位置：

- `navigation/quadruped_nav_bringup/scripts/pcd_to_static_map.py`

当前第一版地图输出目录：

- `navigation/quadruped_nav_bringup/maps/`

## 8. 提交与文档约定

- 不要删除历史问题与历史验证文档
- 过期计划应移动到 `docs/history/`，当前计划集中在 `docs/planning/`
- 如果修改了导航主线，请优先更新 `docs/planning/` 下的对应文档
- 如果只是验证结果落地，优先补充到已有验证记录，而不是新建重复文档

## 9. 一句话总结

当前仓库的关键不是继续调局部闭环，而是把已经验证可行的 `PCD -> 2D static map` 路线正式接到 `map_server -> /map -> global_costmap`。
