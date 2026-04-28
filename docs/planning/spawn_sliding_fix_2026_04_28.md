# 机器狗 Spawn 被动向后滑动修复

## 现象

机器狗在 Gazebo 中 spawn 后，以 PASSIVE 模式（关节 kp=0, 力矩=0）自然趴地，
随即缓慢持续向后滑动。切换控制器 mode:2 (FIXEDDOWN) 后滑动立即停止。

## 根因

ODE 物理引擎的摩擦模型在近零速度下采用速度软化近似（F ∝ v），
不具备真实静摩擦。落地冲击产生微量水平动量后，摩擦力随速度趋零而趋零，
无法将残余动能归零，导致永续蠕变滑动。

这不是参数可修的问题——曾尝试调整接触刚度/阻尼、关节摩擦/限位，
均无法消除 ODE 对静摩擦的建模缺陷。

## 修复方案

将控制器初始状态从 PASSIVE 改为 FIXEDDOWN，
启动即由 PD 控制器（kp=80, kd=3.5）维持趴地姿态，
通过运动链锁定整机位置，杜绝蠕变启动机会。

## 修改

`controllers/unitree_guide_controller/src/UnitreeGuideController.cpp:227`

```diff
- current_state_ = state_list_.passive;
+ current_state_ = state_list_.fixedDown;
```

```diff
- ctrl_interfaces_.motion_command_.requested_state_ = FSMStateName::PASSIVE;
+ ctrl_interfaces_.motion_command_.requested_state_ = FSMStateName::FIXEDDOWN;
```

## 影响

- 启动后机器人直接进入趴地锁定姿态，不滑动
- 趴地角度与 PASSIVE 自然塌落姿态一致（使用同一个 `down_pos` 目标）
- 不影响步态、trotting、导航等任何其他模式
- 如需回到完全被动瘫倒状态，手动按键盘 `1` 即可
