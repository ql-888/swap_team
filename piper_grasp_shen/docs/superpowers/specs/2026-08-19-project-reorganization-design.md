# Piper Grasp Shen 项目整理与本机适配设计

日期：2026-08-19

## 目标

在 `/home/guoyi/gy_ws/piper_grasp_shen` 内完成保守的原地整理，使项目能够在当前 Ubuntu 22.04、ROS 2 Humble 和现有 `piper_pink` Conda 环境下执行离线规划与只读硬件检查，并为后续 D435i 腕部 AprilTag 抓取保留稳定入口。

本次不使能机械臂、不发送运动或夹爪命令。真机运动验收留给人工清场后的独立步骤。

## 设计原则

1. 保留 `src/piper_pink`、`scripts`、`config`、`assets` 和 `tests` 这些标准 Python/机器人项目边界。
2. 不大规模重命名现有脚本入口，避免破坏已经验证过的操作流程。
3. 标定原始结果进入项目内单一目录，并由运行配置使用相对、可迁移的溯源路径。
4. 所有模型资源路径必须与用户名和安装目录无关。
5. Conda 负责 Pinocchio、Pink 和规划依赖；ROS 系统 Python 负责 `rclpy` 和 ROS 消息。脚本必须明确选择正确解释器。
6. 所有修复先有自动化测试，最终同时执行单元测试和离线规划验收。

## 目标结构

```text
piper_grasp_shen/
├── README.md
├── pyproject.toml
├── requirements.lock.txt
├── assets/
├── calibration_data/
│   ├── README.md
│   ├── d435i_eye_in_hand.json
│   └── gemini336l_eye_to_hand.json
├── config/
├── docs/
│   ├── STRUCTURE.md
│   ├── DEVICE_SETUP.md
│   └── superpowers/specs/
├── runtime/
├── scripts/
├── src/piper_pink/
└── tests/
```

## 标定数据

- 将 `/home/guoyi/gy_ws/swap/calibration/calibration_in.json` 复制为 `calibration_data/d435i_eye_in_hand.json`。
- 将 `/home/guoyi/gy_ws/handeye/results/2026-08-18_20-13-17_eye_to_hand_orbbec.json` 复制为 `calibration_data/gemini336l_eye_to_hand.json`。
- 运行外参仍保存在两份简洁 YAML 中，因为加载、坐标变换和 TF 发布依赖其规范化矩阵。
- YAML 的 `source.file` 使用相对项目根目录的路径，不依赖 `/home/guoyi`。
- 自动化测试逐元素比较 JSON 与 YAML，防止整理过程中改变外参。

## 模型资源路径

当前 Piper X URDF 将网格硬编码为 `/home/shen_tao/robot_projects/piper_grasp/...`，并引用项目中不存在的 DAE 文件。修复方式：

- 将视觉和碰撞网格统一改为现有 STL。
- 使用 `package://piper_x_description/meshes/...` URI。
- Pinocchio 加载时继续把 `assets` 作为 package 根目录。
- 新增测试，拒绝 URDF 中出现 `/home/` 绝对路径，并实际构建碰撞模型。

## Python、Conda 与 ROS 边界

- `scripts/run.sh` 默认定位名为 `piper_pink` 的 Conda 环境；允许通过 `PIPER_PINK_PYTHON` 显式覆盖解释器。
- `scripts/setup_env.sh` 更新现有 `piper_pink` 环境，而不是在项目中再创建 `.conda`。
- 核心包通过 `python -m pip install --no-deps -e <project>` 安装到该环境。
- 离线规划、IK、碰撞和测试由 Conda Python 执行。
- 使用 `rclpy` 的 ROS 脚本固定由 `/usr/bin/python3` 执行，并在启动前加载 `/opt/ros/humble/setup.bash`；它们通过文件和 ROS topic 与 Conda 规划进程交互。
- 环境配置前先审计现有版本，只安装缺失或明确不兼容的依赖，不无条件升级整个环境。

## 代码修复范围

1. 修复模型资源 URI，使碰撞检查与离线规划在当前目录可运行。
2. 修复 `plan_standard52_transport.py` 将 4×4 变换误当作三维闭合轴的错误。
3. 调整 ROS 执行脚本解释器和入口检查，使其不会因激活 Conda 而丢失 `rclpy`。
4. 保持 D435i 单相机流程和 Gemini-to-D435i 接力流程的确认字符串、安全速度、位姿新鲜度、跳变限制和夹爪反馈保护不变。
5. 清理可再生成的 `__pycache__` 与 `src/piper_grasp.egg-info`；历史 `runtime` 报告不删除。

## 文档

`docs/STRUCTURE.md` 解释每个目录、哪些文件可以修改、哪些是生成物。`docs/DEVICE_SETUP.md` 提供以下顺序：环境、ROS、相机/CAN、外参 TF、AprilTag、只读识别、离线规划和人工确认后的分阶段真机入口。

## 测试与验收

- 所有 Python 单元测试通过。
- URDF 不包含旧用户名或绝对 `/home` 网格路径。
- Pinocchio 可加载模型和碰撞几何。
- 两份 JSON 与两份运行 YAML 的外参一致。
- `piper_pink` 环境可导入规划、视觉、Piper SDK 和项目包。
- `doctor` 与示例 `plan-grasp` 离线通过。
- ROS 只读检查报告 ROS、相机 topic、TF、机械臂反馈和 `can0` 的实际状态。

若硬件未连接，只读检查可以报告缺失，但不得把“设备缺失”误报为软件通过。若任何测试失败，最终指引必须列出阻塞项，不宣称项目已可真机抓取。

## 非目标

- 不重新计算相机标定。
- 不改变抓取几何、TCP 数值或机械臂速度限制。
- 不自动启动、使能或移动 Piper X。
- 不删除历史运行报告。
- 不安装或修改系统级 ROS、udev、CAN 网络配置，除非后续得到单独授权。
