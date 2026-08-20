# 项目结构指引

项目根目录：`/home/guoyi/gy_ws/piper_grasp_shen`

## 顶层目录

| 路径 | 用途 | 是否手工修改 |
|---|---|---|
| `.conda/` | 项目本地 Python 3.10、Pinocchio、Pink、OpenCV 和 Piper SDK 环境 | 否；使用 `scripts/setup_env.sh` |
| `assets/` | Piper/Piper X URDF、SRDF 和 STL 网格 | 仅在更换机器人模型时 |
| `calibration_data/` | D435i 与 Gemini 336L 当前原始标定 JSON | 仅重新标定后 |
| `config/` | 运行外参、AprilTag 参数、TCP、场景和抓取配方 | 按现场测量修改 |
| `docs/` | 结构、设备启动和设计文档 | 可以 |
| `runtime/` | 检测位姿、规划报告和历史执行记录 | 程序生成；不要作为固定配置 |
| `scripts/` | 环境、ROS、视觉、规划和安全执行入口 | 谨慎修改 |
| `src/piper_pink/` | 模型、IK、碰撞、变换、抓取和 CLI 核心代码 | 可以，修改后运行测试 |
| `tests/` | 离线单元与集成测试 | 与代码同步维护 |

## 标定文件关系

```text
calibration_data/d435i_eye_in_hand.json
        └── config/handeye_d435i_wrist_end_pose.yaml

calibration_data/gemini336l_eye_to_hand.json
        └── config/handeye_orbbec_fixed.yaml
```

JSON 保存完整样本、质量指标和原始结果；YAML 保存运行时需要的规范化矩阵及坐标系名称。
测试会逐元素比较两者。抓取流程使用 YAML，不会在启动时重新计算标定。

## 稳定入口

- `scripts/run.sh`：使用项目 `.conda` 运行核心 CLI。
- `scripts/setup_env.sh`：验证依赖并把当前源码重新安装到项目环境。
- `scripts/verify_offline.sh`：完整离线测试与示例规划，不连接 CAN。
- `scripts/plan_d435i_apriltag_grasp.sh`：D435i 采集后只规划。
- `scripts/run_full_d435i_apriltag_grasp_retreat.sh`：D435i 完整抓取入口，会运动。
- `scripts/run_global_to_wrist_apriltag_grasp_retreat.sh`：Gemini 粗定位到 D435i 精定位，会运动。

`runtime/` 中的 YAML 是某一次运行的快照，不应复制到 `config/` 充当永久配方。
`.conda/`、`__pycache__/`、`.pytest_cache/` 和 egg-info 都是可再生成内容。
