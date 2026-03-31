# FAST-LIVO2 原生 PCD 落盘说明（2026-03-31）

本文档用于说明当前仓库中 FAST-LIVO2 的原生 PCD 落盘能力、相关参数、输出路径以及推荐使用方式。

本文档的目标不是讨论静态地图如何最终接入 Nav2，而是先把“如何稳定导出一份全局点云原料”这件事说明清楚。

## 一、当前结论

当前仓库中的 FAST-LIVO2 已经具备原生 PCD 落盘能力，无需优先依赖外部录包或额外挂接点云保存节点。

也就是说，当前更推荐：

- 直接启用 FAST-LIVO2 自带的 `pcd_save`
- 让 FAST-LIVO2 在运行过程中累计全局点云
- 由其原生逻辑输出 PCD 文件

这样做的优势是：

- 输出路径清晰
- 与 FAST-LIVO2 内部全局点云表达一致
- 不需要先通过 ROS 话题再做二次导出
- 更容易控制 frame 语义的一致性

## 二、相关代码与配置位置

### 1. 配置文件

当前 Gazebo 仿真启动使用的主配置文件是：

- `src/lite3_gazebo_classic/src/FAST-LIVO2/config/mid360_sim.yaml`

其中已存在 `pcd_save` 配置段：

```yaml
pcd_save:
  pcd_save_en: false
  colmap_output_en: false
  filter_size_pcd: 0.15
  interval: -1
```

### 2. 启动文件

当前仿真使用的 launch 文件是：

- `src/lite3_gazebo_classic/src/FAST-LIVO2/launch/mapping_gazebo.launch.py`

该 launch 默认加载：

- `mid360_sim.yaml`
- `camera_sim.yaml`

因此，只要修改 `mid360_sim.yaml` 中的 `pcd_save` 参数，就能直接影响仿真中的 FAST-LIVO2 落盘行为。

### 3. 源码逻辑

原生保存逻辑位于：

- `src/lite3_gazebo_classic/src/FAST-LIVO2/src/LIVMapper.cpp`

其中已经实现了：

- 参数声明与读取
- 点云累计
- 原始 PCD 保存
- 下采样 PCD 保存
- 按单文件或分段文件输出

## 三、参数含义

### `pcd_save.pcd_save_en`

是否启用 PCD 落盘。

- `false`：不保存
- `true`：启用保存

### `pcd_save.interval`

决定 PCD 的输出方式。

- `-1`：运行期间持续累计，最终保存为单个全局 PCD
- `> 0`：每累计若干帧保存一次，生成多个分段 PCD 文件

当前更推荐第一版使用：

- `interval: -1`

原因是当前目标是得到一份静态地图原料，而不是分段扫描集。

### `pcd_save.filter_size_pcd`

控制保存时下采样点云的体素滤波尺寸。

当前默认值：

- `0.15`

它会影响 `all_downsampled_points.pcd` 的密度。

### `pcd_save.colmap_output_en`

是否额外输出 COLMAP 相关格式。

当前静态地图链路不依赖此功能，因此建议继续保持：

- `false`

## 四、输出文件路径

根据当前源码逻辑，输出路径位于 FAST-LIVO2 仓库自身目录下：

- `src/lite3_gazebo_classic/src/FAST-LIVO2/Log/PCD/`

### 当 `interval < 0` 时

FAST-LIVO2 会在保存阶段输出：

- `all_raw_points.pcd`
- `all_downsampled_points.pcd`

含义分别为：

- `all_raw_points.pcd`：累计后的原始全局点云
- `all_downsampled_points.pcd`：经过体素滤波后的全局点云

对于静态地图制备，通常更建议先从：

- `all_downsampled_points.pcd`

开始做第一版验证，因为它更轻，后续做 2D 投影时更容易处理。

### 当 `interval > 0` 时

FAST-LIVO2 会输出：

- `1.pcd`
- `2.pcd`
- `3.pcd`
- ...

同时还会记录对应位姿信息。

这更适合后续做分段扫描处理，不适合作为当前第一版静态地图主路径。

## 五、当前 frame 语义说明

这是当前最需要明确的一点。

根据实际话题核对结果，当前 FAST-LIVO2 输出中：

- `/cloud_registered_lidar.header.frame_id = camera_init`
- `/aft_mapped_to_init.header.frame_id = camera_init`
- `/aft_mapped_to_init.child_frame_id = aft_mapped`

同时，源码中发布全局点云时也明确将点云 frame 设为：

- `camera_init`

因此，当前原生导出的 PCD 应理解为：

- 处于 FAST-LIVO2 的全局固定参考系 `camera_init` 下

当前不要直接把这份 PCD 口头等同为最终 `/map`。

更严谨的说法应是：

- 这是一份以 `camera_init` 为全局参考系的静态点云原料
- 后续是否将其对齐到真正 `/map`，需要在静态地图链和 `map -> odom` 设计中统一处理

## 六、推荐使用方式

当前推荐的第一版流程如下：

1. 修改 `mid360_sim.yaml`，开启：
   - `pcd_save.pcd_save_en: true`
2. 保持：
   - `pcd_save.interval: -1`
3. 启动 Gazebo 与 FAST-LIVO2 仿真
4. 控制机器人覆盖需要建图的主要区域
5. 正常结束 FAST-LIVO2 节点
6. 到 `FAST-LIVO2/Log/PCD/` 目录检查输出结果
7. 优先取 `all_downsampled_points.pcd` 作为静态地图第一版输入

## 七、为什么优先使用原生导出

与外部录制 ROS 话题相比，当前更推荐优先使用 FAST-LIVO2 原生导出的原因包括：

- 不需要额外增加导出节点
- 不会在 bridge 或 Nav2 侧引入额外 frame 重映射
- 更接近 FAST-LIVO2 内部实际维护的全局点云表示
- 后续排查静态地图质量问题时，边界更清晰

也就是说，当前这一步的核心目标不是“最通用”，而是“最一致、最省事、最少引入新变量”。

## 八、当前不建议的做法

在当前阶段，不建议优先采用以下路线：

- 从 `/obstacle_points` 再反向导出 PCD
- 从 `floor_mapper` 结果反推出静态地图
- 在 bridge 尚未最终收敛前，把桥接后的 frame 当成最终全局地图语义

这些方式不是永远不能做，而是当前不应作为第一版静态地图原料获取路径。

## 九、当前阶段结论

截至目前，最清晰、最省事、最符合当前仓库结构的全局点云落盘路径是：

- 使用 FAST-LIVO2 原生 `pcd_save`
- 在 `mid360_sim.yaml` 中开启保存
- 采用 `interval: -1` 导出单份全局 PCD
- 优先使用 `all_downsampled_points.pcd` 作为后续静态 2D 地图制备输入

在这一步完成后，下一阶段再进入：

- 3D PCD 到 2D 地图的离线投影/切片
- `map_server` 接入
- `map -> odom` 正式语义收敛
