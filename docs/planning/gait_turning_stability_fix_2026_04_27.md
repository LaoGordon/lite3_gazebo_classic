# 步态大幅度转向稳定性修复

日期: 2026-04-27

## 问题描述

四足机器人在导航给定目标点转向（大幅度 yaw 转向）时步态不稳定，表现为：
1. 腿不摆动，身子拧转后摔倒
2. 直行时走路不稳，误差累积导致侧翻

## 根因分析

共发现 6 处问题，按严重程度排序：

### 根因 1 — WaveGenerator 步态切换将腿锁死在 stance（核心故障）

**文件:** `controllers/unitree_guide_controller/src/gait/WaveGenerator.cpp:39-49`

`WaveGenerator::update()` 在 `STANCE_ALL → WAVE_ALL` 切换时，没有像 `SWING_ALL ↔ STANCE_ALL` 那样的立即同步逻辑，而是走默认的 `switch_status_` 渐进切换。那些恰好处在 swing 相的腿会被**强制锁在 stance（contact=1, phase=0.5）**，要等半个 gait 周期才能释放。

**后果:** 导航转向时指令频繁变化，反复触发 `checkStepOrNot()` 的 `STANCE_ALL ↔ WAVE_ALL` 切换。腿被钉在地上抬不起来，QP 在 4 条腿全着地状态下硬拧机身 → 身子动脚不动 → 扭 → 摔倒。

**修复:** `STANCE_ALL → WAVE_ALL` 切换时立即同步，跳过渐进切换：
```cpp
if (status_ == WaveStatus::WAVE_ALL && status_past_ == WaveStatus::STANCE_ALL) {
    status_past_ = status_;
    switch_status_.setZero();
}
```

---

### 根因 2 — FeetEndCalc 足端角度用世界坐标系导致 yaw 重复计入

**文件:** `controllers/unitree_guide_controller/src/gait/FeetEndCalc.cpp:23-29`

`init()` 调用 `estimator_->getFeetPos2Body()` 获取足端位置——但该函数返回的是**世界坐标系**下的值（`rotation_ * foot_body`），其极角包含机身 yaw。后续 `calcFootPos()` 中又加了当前 yaw：
```cpp
cos(yaw_current + feet_init_angle_ + next_yaw)
= cos(yaw_current + yaw_init + body_angle + next_yaw)  // yaw_init 重复计入
```

**后果:** 机器人转向角度越大，落足点系统性偏移越大。导航中累积转向 90° 时，落足点偏差可达 1.57 rad。

**修复:** 改用 `robot_model_->getFeet2BPositions()` 获取**body 坐标系**的足端位置，`feet_init_angle_` 只包含 body 帧角度。

---

### 根因 3 — Estimator 足端速度坐标系混用

**文件:** `controllers/unitree_guide_controller/include/unitree_guide_controller/control/Estimator.h:69`

`getFeetVel()` 将 body 帧的足端相对速度直接与世界帧的机身速度相加：
```cpp
result.col(i) = Vec3(feet_vel[i].data) + getVelocity();
//                body 帧                    world 帧
```

缺少 `rotation_` 旋转，导致足端速度方向在转向时错误。

**后果:** 摆动腿阻抗控制的微分项（`Kd_swing_`）使用错误的速度误差计算出力，误差随转向角度累积，最终侧翻。

**修复:** 加上 `rotation_ *` 将 body 帧速度转到 world 帧：
```cpp
result.col(i) = rotation_ * Vec3(feet_vel[i].data) + getVelocity();
```

---

### 根因 4 — GaitGenerator 摆线终点持续漂移

**文件:** `controllers/unitree_guide_controller/src/gait/GaitGenerator.cpp:41`

`generate()` 在 swing 期间每帧都重算 `end_p_`，但摆线插值要求起点终点固定。大幅转向时 yaw rate 和 body velocity 在 swing 期间剧烈变化，导致终点漂移，轨迹扭曲。

**修复:** 仅在 swing 开始时（`prev_contact_ == 1`，即刚离地）一次性计算 `end_p_`，之后保持不变。

---

### 根因 5 — yaw 角加速度饱和限过严

**文件:** `controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp:138`

`d_wbd(2)` yaw 角加速度饱和为 ±10 rad/s²，而 roll/pitch 为 ±40。配合 `kp_w_ = 780` 的高增益，yaw 误差超过 0.013 rad 就饱和，姿态控制几乎一直处于限幅状态。

**修复:** yaw 饱和与 roll/pitch 对等，统一为 ±40 rad/s²。

---

### 根因 6 — yaw 落足预测反馈增益过小

**文件:** `controllers/unitree_guide_controller/src/gait/FeetEndCalc.cpp:16`

`k_yaw_ = 0.005`，当 yaw 指令 0.2 rad/s 而实际为 0 时，修正项仅 0.001 rad（占前馈项的 2%），落足点几乎不响应 yaw 跟踪误差。

**修复:** `k_yaw_` 提升到 0.05，修正量提升到 ~20%。

---

## 修改文件清单

| # | 文件 | 改动行 | 类型 |
|---|------|--------|------|
| 1 | `WaveGenerator.cpp:39-49` | `STANCE_ALL→WAVE_ALL` 立即同步 | 逻辑错误 |
| 2 | `FeetEndCalc.cpp:23-29` | `init()` 用 body 帧足端位置 | 坐标系混用 |
| 3 | `Estimator.h:69` | `getFeetVel()` body→world 帧旋转 | 坐标系混用 |
| 4 | `GaitGenerator.cpp:41-44,49` | `end_p_` swing 期间只算一次 | 轨迹漂移 |
| 5 | `StateTrotting.cpp:138` | `d_wbd(2)` 饱和 ±10→±40 | 参数保守 |
| 6 | `FeetEndCalc.cpp:16` | `k_yaw_` 0.005→0.05 | 参数保守 |

## 验证结果

- 大幅度导航转向不再摔倒
- 直行稳定性恢复
- 整体 gait 协调性改善
