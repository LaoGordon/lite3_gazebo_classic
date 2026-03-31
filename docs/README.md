# Lite3 Gazebo Classic 文档中心

本文档目录包含 Lite3 Gazebo Classic 仿真环境的使用说明、导航设计文档以及历史验证记录。

## 使用说明

| 文档 | 说明 |
|------|------|
| [传感器配置](./guides/sensor_configuration.md) | 当前传感器配置与话题说明 |
| [键盘控制指南](./guides/keyboard_control_guide.md) | 当前键盘控制使用说明 |
| [故障排除](./guides/troubleshooting.md) | 当前常见问题与处理办法 |

## 参考资料

| 文档 | 说明 |
|------|------|
| [控制接口规范](./references/control_interface_spec.md) | 当前稳定外部控制接口基线 |
| [FAST-LIVO2 Rcl 分析](./references/fastlivo2_rcl_analysis.md) | 当前 LiDAR-相机外参分析记录 |

## 当前规划文档

| 文档 | 说明 |
|------|------|
| [导航当前问题记录](./planning/navigation_issues_2026_03_31.md) | 当前已识别的导航架构问题、风险边界与处理顺序 |
| [静态Map链路设计草案](./planning/static_map_pipeline_design_2026_03_31.md) | 当前从全局点云落盘到 `/map` 接入 Nav2 的设计路径 |
| [FAST-LIVO2原生PCD落盘说明](./planning/fastlivo2_pcd_export_guide_2026_03_31.md) | 当前原生全局点云导出的参数、路径与推荐使用方式 |
| [PCD到2D静态地图验证记录](./planning/pcd_to_map_validation_2026_03_31.md) | 第一版离线静态地图生成结果与人工检查结论 |

## 历史验证记录

| 文档 | 说明 |
|------|------|
| [导航闭环验证记录](./history/navigation_validation_2026_03_24.md) | 2026-03-24 的最小闭环实际验证过程与结果 |
| [导航当前状态总结](./history/navigation_status_2026_03_25.md) | 2026-03-25 的阶段状态总结，保留历史结论与当时计划 |
| [导航集成联调进展](./history/navigation_progress_2026_03_30.md) | 2026-03-30 的阶段进展记录，保留历史问题分析与当时计划 |

## 使用建议

如果你现在要继续开发或排查问题，建议优先阅读：

1. [导航当前问题记录](./planning/navigation_issues_2026_03_31.md)
2. [静态Map链路设计草案](./planning/static_map_pipeline_design_2026_03_31.md)
3. [FAST-LIVO2原生PCD落盘说明](./planning/fastlivo2_pcd_export_guide_2026_03_31.md)
4. [导航闭环验证记录](./history/navigation_validation_2026_03_24.md)

历史文档不会删除，因为其中保留了：

- 已解决问题的背景
- 实际联调过程
- 重要设计演变原因

但凡涉及“下一步计划”，应优先以 2026-03-31 的规划文档为准。
