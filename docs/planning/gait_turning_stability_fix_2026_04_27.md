# 步态转向稳定性修复 —— 最终版

日期: 2026-04-27

---

## 成果总览

本次会话解决了四足机器人导航中的两个核心问题：

| 成果 | 描述 |
|------|------|
| **里程碑 1：大幅度转向不再翻车** | 根因：WaveGenerator 步态切换将腿锁死 |
| **里程碑 2：步态收敛到最稳定版本** | 经 8 轮迭代，去伪存真，留下 6 项修复 + Nav2 参数对齐 |

---

## 里程碑 1：大幅度转向不再翻车

### 症状

导航给定目标点转向时，四足机器人 "身子动脚不动，在那里扭，然后摔倒"。

### 根因

**`WaveGenerator::update()` 在 `STANCE_ALL → WAVE_ALL` 切换时没有立即同步逻辑**。

导航中 `/cmd_vel` 频繁变化，反复触发 `checkStepOrNot()` 的状态切换。`switch_status_` 渐进机制将部分腿**强制锁死在 stance（contact=1, phase=0.5）**，要等半个 gait 周期才能释放。QP 在 4 条腿全着地状态下硬拧机身 → 扭 → 摔倒。

**文件:** `controllers/unitree_guide_controller/src/gait/WaveGenerator.cpp:39`

**修复:** 新增 `STANCE_ALL → WAVE_ALL` 立即同步分支：

```cpp
if (status_ == WaveStatus::WAVE_ALL && status_past_ == WaveStatus::STANCE_ALL) {
    status_past_ = status_;
    switch_status_.setZero();  // 跳过渐进切换，所有腿立即跟随波形
}
```

这是整个会话中**最关键的一项改动**。仅此一项，大幅度转向就不再摔倒。

### 支撑性修复

以下两项坐标系混用 bug 在转向时才暴露，修复后进一步消除误差累积：

| # | 文件 | 问题 | 修复 |
|---|------|------|------|
| 2 | `FeetEndCalc.cpp:init()` | `getFeetPos2Body()` 返回世界帧值，`feet_init_angle_` 嵌入了 yaw_init，后续 `calcFootPos` 又加一次当前 yaw → yaw 重复计入 | 改用 `robot_model_->getFeet2BPositions()` 获取 body 帧足端位置 |
| 3 | `Estimator.h:getFeetVel()` | body 帧足端速度直接加 world 帧机身速度，缺少 `rotation_` 旋转 | 加 `rotation_ *` 将足端速度转到 world 帧再相加 |

---

## 里程碑 2：步态收敛到最稳定版本

### 迭代过程

在转向不翻车之后，走路仍不稳定——表现为 "转一次，退几步来稳定自己"、整体绕 Z 轴旋转。经过以下迭代：

| 轮次 | 改动 | 效果 | 结论 |
|------|------|------|------|
| 1 | `k_yaw_`: 0.005→0.05 | 转向响应增强 | 走路时中心轴旋转 |
| 2 | `d_wbd(2)` 饱和: ±10→±40 | yaw 力矩不再限制 | 配合 k_yaw_ 过激进 |
| 3 | `GaitGenerator`: `end_p_` swing 期间只算一次 | 摆线终点固定 | 过渡优化，反而不稳 |
| 4 | `getFootVel()` 加机身速度 | 速度误差修正 | **恶化**，立即回退 |
| 5 | QP 失败保护 | 防止垃圾力输出 | 保留（QP 实际未失败） |
| 6 | `k_yaw_`: 0.05→0.01→0.005 | 逐步回调 | 回到原始值最稳 |
| 7 | `d_wbd(2)`: ±40→±20→±10 | 逐步回调 | 回到原始值最稳 |
| 8 | 回退 `end_p_` 固定 | 恢复反应式落足 | **明显改善** |

### 最终保留的 6 项修复

| # | 文件 | 改动 | 类型 |
|---|------|------|------|
| 1 | `WaveGenerator.cpp` | `STANCE_ALL→WAVE_ALL` 立即同步 | **核心：转向不再翻车** |
| 2 | `FeetEndCalc.cpp` | `init()` 用 body 帧足端位置 | 坐标系混用修复 |
| 3 | `Estimator.h` | `getFeetVel()` body→world 旋转 | 坐标系混用修复 |
| 4 | `FeetEndCalc.cpp` | `k_yaw_` = 0.005（原始值） | 确认原始值正确 |
| 5 | `StateTrotting.cpp` | `d_wbd(2)` = ±10（原始值） | 确认原始值正确 |
| 6 | `BalanceCtrl.cpp` | QP 无解时沿用上一帧力 | 防御性保护 |

### 回退的改动及原因

| 改动 | 原因 |
|------|------|
| `end_p_` swing 期间固定（只算一次） | 原始的反应式落足能吸收 gait 固有波动，固定后反而失去适应性 |
| `getFootVel()` 加机身速度 | 与 `calcQQd()` 的 `-vel_body_` 形成双重补偿，关节速度目标偏大 |

### Nav2 参数对齐

| 参数 | 前 → 后 | 原因 |
|------|---------|------|
| `rotate_to_heading_angular_vel` | 1.8 → 0.2 | 匹配机器人 0.2 rad/s 实际能力 |
| `velocity_smoother` 角速度/加速度 | 1.0/2.5 → 0.2/1.0 | 不再输出超限指令 |
| `behavior_server` 旋转限速 | 1.0/0.4/3.2 → 0.2/0.1/1.0 | 行为树对齐 |
| `yaw_goal_tolerance` | 0.25 → 0.4 | 放宽到位精度，避免末端振荡 |
| `planner_frequency` | 20 → 5 Hz | 减少无效重规划 |
| `controller_frequency` | 20 → 10 Hz | 降低控制回环干扰 |

---

## 关键经验

1. **不要同时调多个参数**：`k_yaw_` 和 `d_wbd` 同时提高导致走路旋转，但分开看都合理
2. **反应式 > 预规划式**：原始代码 swing 期间每帧更新 `end_p_` 是有道理的，固定化是过度优化
3. **坐标系 bug 最隐蔽**：`getFeetVel()` 和 `init()` 的 frame 混用在 yaw≈0 时碰巧正确，转向时才会暴露
4. **先解决摔跟头，再调走路**：里程碑 1（WaveGenerator）是致命故障，解决后才能看到走路稳定性问题

---

## 修改文件清单（最终版）

```
controllers/unitree_guide_controller/src/gait/WaveGenerator.cpp    # 修复 1
controllers/unitree_guide_controller/src/gait/FeetEndCalc.cpp       # 修复 2, 4
controllers/unitree_guide_controller/include/.../Estimator.h        # 修复 3
controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp      # 修复 5
controllers/unitree_guide_controller/src/control/BalanceCtrl.cpp    # 修复 6
navigation/quadruped_nav_bringup/config/nav2_params.yaml           # Nav2 参数
```
