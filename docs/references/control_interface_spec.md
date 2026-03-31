# Lite3 Control Interface Spec

This document defines the stable external control interface for the Lite3 simulation stack.

## Standard Topics

The system exposes exactly two standard control topics:

| Topic | Message Type | Purpose |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Motion command |
| `/robot_mode` | `std_msgs/msg/Int32` | Main state transition command |

All teleoperation inputs should publish to these topics. New controllers should reuse this interface unless they require a genuinely different command model.

## Mode Definition

The current main-chain mode mapping is:

| Value | Mode | Description |
|------|------|------|
| `1` | `PASSIVE` | Disable active behavior |
| `2` | `FIXEDDOWN` | Fold or crouch posture |
| `3` | `FIXEDSTAND` | Stand posture |
| `4` | `TROTTING` | Locomotion mode |

The main external control path officially supports the four modes above.

## Velocity Semantics

`/cmd_vel` uses standard mobile-base semantics:

| Field | Meaning |
|------|------|
| `linear.x` | forward / backward velocity command |
| `linear.y` | lateral velocity command |
| `angular.z` | yaw rate command |

The current controller applies these nominal limits:

| Field | Limit |
|------|------|
| `linear.x` | `[-0.4, 0.4]` |
| `linear.y` | `[-0.3, 0.3]` |
| `angular.z` | `[-0.2, 0.2]` |

These limits are controller-side behavior, not a promise that all producers must clip in exactly the same way.

## Supported Input Nodes

The following input nodes already publish the standard interface:

| Package | Output |
|------|------|
| `keyboard_input` | `/cmd_vel`, `/robot_mode` |
| `joystick_input` | `/cmd_vel`, `/robot_mode` |
| `unitree_joystick_input` | `/cmd_vel`, `/robot_mode` |

## Scope Boundary

The standard interface is intended for the main locomotion chain:

- `PASSIVE`
- `FIXEDDOWN`
- `FIXEDSTAND`
- `TROTTING`

Experimental or development-only modes may use internal mappings, but they should not redefine the external standard interface without updating this document.
