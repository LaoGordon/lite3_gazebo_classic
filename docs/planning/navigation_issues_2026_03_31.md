# 导航当前问题记录（2026-03-31）

本文档用于记录截至 2026 年 3 月 31 日，在 Lite3 Gazebo Classic 导航链路中已经识别出的关键问题、风险边界以及建议处理顺序。

本文档的目的不是替代既有验证记录，而是把“当前最需要继续处理的事项”单独整理出来，避免被已有进展文档淹没。

## 一、当前结论

截至目前，可以认为以下结论同时成立：

- 最小导航闭环已经实际打通
- `FAST-LIVO2 -> fastlivo_nav_bridge -> floor_mapper -> Nav2 -> /cmd_vel -> robot motion` 这条链路已经做过最小验证
- 当前主要问题已经从“能不能接上”转为“架构分层是否正确、输入语义是否严谨、系统是否稳定”

因此，后续工作的重点不应继续停留在“证明链路存在”，而应转向：

- 修正明确的架构错配
- 固化输入语义
- 为后续静态全局地图与多楼层扩展预留正确边界

## 二、已识别的关键问题

### 1. `global_costmap` 曾误用 `floor_mapper` 的输出

此前 `nav2_params.yaml` 中，`global_costmap` 曾将 `/floor_map` 作为 `StaticLayer` 输入使用。

这是一处已经识别并已开始修正的架构错配。

原因是当前 `floor_mapper` 的实现并不是全局地图生成器，而是一个局部滚动式障碍投影模块。其主要特征包括：

- 地图为滚动窗口
- 窗口尺寸固定为 20m x 20m
- 仅保留最近一小段时间内的点云历史
- 输出结果更接近局部障碍投影层，而不是长期稳定的全局楼层地图

因此，`/floor_map` 更适合作为局部避障输入，而不是全局路径规划输入。

如果继续将其喂给 `global_costmap`，会导致：

- 全局规划器只能看到机器人附近一小块区域
- 远处障碍、拐角、封闭结构无法被稳定纳入规划
- 全局路径可能建立在错误或不完整的环境认知之上

### 2. `floor_mapper` 的名字已开始偏离实际职责

从当前实现看，`floor_mapper` 这个包已经不太像传统意义上的“楼层地图构建器”，而更像：

- 局部障碍投影器
- 点云到 2D 栅格的近场转换层
- 供导航局部层使用的动态环境输入模块

这个命名会带来两个问题：

- 容易让人误以为 `/floor_map` 是可直接用于全局规划的楼层地图
- 容易在后续真正引入静态楼层地图时与新模块职责冲突

短期内不必立刻改包名，但文档和配置层必须明确：

- 当前 `floor_mapper` 不等于全局地图系统
- 当前 `/floor_map` 不应被默认理解为长期稳定的楼层底图

### 3. `fastlivo_nav_bridge` 的点云 frame 处理仍需继续核对

当前 `fastlivo_nav_bridge_node.cpp` 中，对输入点云采用的是“复制消息并直接覆盖 `header.frame_id`”的做法。

这在当前最小闭环中可能暂时可用，但从工程语义上仍然存在风险：

- 当前代码没有对点云执行显式 TF 变换
- 当前代码依赖于上游点云 frame 语义与导航层 frame 约定之间的隐含一致性
- 一旦后续引入真正的 `map -> odom` 关系，现有做法可能造成点云解释偏差

因此，这个问题需要记录为“后续必须核对”的输入语义问题，但不建议在尚未确认上游原始 frame 含义前直接拍脑袋修改。

### 4. `local_costmap` 的当前实现是否需要改动，尚不应过早定性

当前 `local_costmap` 仍在消费原始障碍点云。

这是否是最优方案，还需要后续结合实际表现继续评估，例如：

- 算力消耗是否可接受
- 局部避障稳定性是否满足需求
- 是否有必要改为直接消费 `/floor_map`

但截至目前，这个问题更适合作为“后续优化与验证项”，而不是当前第一优先级的确定性错误。

换句话说：

- `global_costmap <- /floor_map` 是当前最明确的架构问题
- `local_costmap` 的具体实现路线，当前仍应保留验证空间

## 三、当前建议的处理顺序

### 第一优先级：修正全局地图输入错配

首先应处理：

- 让 `global_costmap` 停止直接使用 `/floor_map`

这一步的目的不是立刻得到理想的全局规划，而是先停止让错误类型的输入继续污染全局规划结果。

在真正的静态全局地图链路尚未完成前，可以接受一个过渡状态：

- 全局层先不依赖 `/floor_map`
- 后续再接入真正的 `/map` 与 `map_server`

### 第二优先级：明确 `floor_mapper` 的职责边界

应在文档和后续设计中明确：

- `floor_mapper` 当前是局部导航输入模块
- 它不是长期静态楼层地图生产者

如果后续架构稳定，再决定是否重命名 package、topic 或配置项。

### 第三优先级：核对 `fastlivo_nav_bridge` 输入语义

在修改 bridge 之前，应先核对以下事实：

- `/cloud_registered_lidar` 的原始 `header.frame_id`
- `/aft_mapped_to_init` 的 `header.frame_id`
- `/aft_mapped_to_init` 的 `child_frame_id`
- 当前 TF 树中 FAST-LIVO 相关 frame 与 `map / odom / base` 的关系

在这些事实确认之前，不建议直接重构 bridge 的点云 frame 处理逻辑。

### 第四优先级：建立真正的静态全局地图链路

后续若要支撑更远距离导航、拐角路径规划和多楼层扩展，应建立：

- 离线或半离线全局点云落盘流程
- 3D 点云到 2D 静态地图的制备流程
- `map_server` 驱动的 `/map` 分发链路

只有在这个链路建立之后，`global_costmap` 才会重新具备正确的长期输入来源。

## 四、2026-03-31 实际核对结果

### 1. FAST-LIVO2 上游原始 frame 已确认

本次实际核对结果表明：

- `/cloud_registered_lidar.header.frame_id = camera_init`
- `/aft_mapped_to_init.header.frame_id = camera_init`
- `/aft_mapped_to_init.child_frame_id = aft_mapped`

这说明 FAST-LIVO2 当前对外发布的点云与里程计，并不是直接使用 `map` 或 `odom` 作为原始参考系，而是使用其自身的 `camera_init` 语义。

### 2. 当前导航桥接实际采用的是“语义重映射”而非显式 TF 点云变换

结合当前 bridge 实现与实际观测，可以认为当前链路的工作方式是：

- FAST-LIVO2 原始里程计先以 `camera_init` 语义输出
- `fastlivo_nav_bridge` 将其整理为导航侧的 `/odom` 与 `odom -> base`
- 障碍点云在 bridge 中被直接改写 `header.frame_id` 后发布给导航层
- 同时 bridge 还发布恒等的 `map -> odom`

因此，当前系统之所以能工作，并不是因为已经严格完成了点云到导航目标 frame 的显式 TF 变换，而是因为当前实现里近似把：

- `camera_init`
- `odom`
- `map`

在最小闭环场景下视作同一全局参考语义使用。

### 3. 当前 bridge 结论：短期可继续使用，中期必须与真正 `/map` 链路一起重构

基于本次核对结果，可以得出更准确的结论：

- 当前 `fastlivo_nav_bridge` 不是一个已经导致系统立刻失效的阻塞性 bug
- 当前实现能支撑最小闭环继续联调
- 但其点云 frame 处理方式仍属于被 `identity map -> odom` 暂时掩盖的语义债务

这意味着：

- 在当前最小闭环阶段，可以暂不立即重构 bridge
- 一旦后续引入真正的静态 `/map`，使 `map -> odom` 不再是恒等关系，就必须重新定义 bridge 的点云输出语义

届时需要在以下方案中做明确选择：

- 保持障碍点云在 `odom` 语义下输出
- 或显式将点云从 `camera_init` 变换到目标导航 frame

但无论采用哪种方案，都不应再继续依赖当前“直接覆盖 frame_id”的方式长期使用。

## 五、当前不建议做的事情

为减少联调扰动，当前阶段不建议同时进行以下多项高耦合修改：

- 一次性同时大改 `global_costmap`、`local_costmap` 和 `bridge`
- 在未核对 frame 语义前直接重写 bridge 的点云转换逻辑
- 因命名不满意而立刻重命名 `floor_mapper` 包及其所有引用

这些动作并非永远不做，而是不适合作为当前第一步动作。

## 六、当前阶段建议

当前最务实的行动建议是：

1. 先修正 `global_costmap` 对 `/floor_map` 的错误依赖
2. 先不同时改动 `local_costmap`
3. 记录并核对 FAST-LIVO2 到 bridge 的 frame 语义
4. 再推进真正的 `/map` 静态地图链路
5. 最后再决定是否重命名 `floor_mapper` 以及是否调整局部层实现

这一路线的核心原则是：

- 先修确定性错误
- 再处理优化问题
- 最后处理命名与扩展性问题

## 七、待办修复计划 (TODO List)

1. 修改 `src/lite3_gazebo_classic/navigation/quadruped_nav_bringup/config/nav2_params.yaml` 的 `global_costmap`，移除对 `/floor_map` 的 `static_layer` 依赖。
2. 保持 `global_costmap` 处于最小过渡配置，不在这一步新增 `rolling_window`、`obstacle_layer` 或新的全局点云输入。
3. 验证 `quadruped_nav_bringup` 仍可正常启动，且最小近距离闭环不因这次修改直接失效。
4. 核对 FAST-LIVO2 上游事实：
    - `/cloud_registered_lidar.header.frame_id`
    - `/aft_mapped_to_init.header.frame_id`
    - `/aft_mapped_to_init.child_frame_id`
    - 当前 TF 树中相关 frame 与 `map / odom / base` 的关系
5. 基于上述事实，再决定是否修改 `src/lite3_gazebo_classic/navigation/fastlivo_nav_bridge/src/fastlivo_nav_bridge_node.cpp`，以及是否需要引入 `tf2` 显式点云变换。
6. 设计真正的静态全局地图链路：
    - 全局 3D 点云落盘
    - 离线 2D 投影/切片
    - 生成 `.pgm` + `.yaml`
    - bringup 中接入 `map_server`
7. 在静态 `/map` 可用后，再把 `global_costmap` 切回标准 `StaticLayer` -> `/map` 结构。
8. 重新评估 `local_costmap` 是继续保留点云层，还是改为更多依赖 `/floor_map`。
9. 架构稳定后，再决定是否重命名 `floor_mapper`，避免它继续被误解为全局楼层地图模块。
