# 当前设备运行指引

以下命令默认从项目目录执行。除明确标记为“会运动”的入口外，检查步骤不会使能机械臂。

## 1. 项目环境

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/setup_env.sh
./scripts/run.sh info
./scripts/verify_offline.sh
```

环境在项目内 `.conda/`。普通规划命令使用 `scripts/run.sh`，不需要手工激活环境。

## 2. ROS 2 与 Piper 工作空间

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash
source /home/guoyi/agx_arm_ws/install/setup.bash
ros2 pkg prefix piper_description
ros2 pkg prefix agx_arm_ctrl
ros2 pkg prefix tf2_ros
```

项目脚本默认叠加使用 `/home/guoyi/gy_ws/handeye/install/setup.bash` 和
`/home/guoyi/agx_arm_ws/install/setup.bash`。如果工作空间移动，执行前分别设置
`PIPER_ROS_SETUP=/实际路径/install/setup.bash` 和
`AGX_ARM_ROS_SETUP=/实际路径/agx_arm_ws/install/setup.bash`。

## 3. USB 与 CAN 只读检查

```bash
lsusb
ip -details -statistics link show can0
./scripts/run.sh doctor --require-can
./scripts/run.sh read-arm --can can0
```

`read-arm` 只读取反馈。若 `can0` 不存在，使用 `scripts/setup_can.sh` 会修改网络接口，
应确认 USB-CAN 型号和 1 Mbit/s 波特率后再单独执行。

## 4. D435i 腕部相机

先启动 RealSense ROS 驱动，并确认彩色图像与内参：

```bash
ros2 topic list | grep d435i
ros2 topic hz /d435i/color/image_raw
ros2 topic echo --once /d435i/color/camera_info
```

随后分别启动手眼 TF 和 AprilTag：

```bash
./scripts/publish_d435i_handeye_tf.sh
./scripts/run_apriltag_d435i.sh
ros2 run tf2_ros tf2_echo piper_x/base_link apriltag_0
```

只采集并规划，不使能机械臂：

```bash
./scripts/plan_d435i_apriltag_grasp.sh
```

## 5. Gemini 336L 全局相机

先启动 Orbbec ROS 驱动，再确认实际 topic 与 TF：

```bash
ros2 topic list | grep -E 'gemini|camera'
./scripts/publish_gemini336l_handeye_tf.sh
./scripts/run_apriltag_gemini336l.sh
ros2 run tf2_ros tf2_echo piper_x/base_link gemini_apriltag_0
```

Gemini 用于全局粗定位，D435i 用于接近后的局部精定位。两台相机的 AprilTag TF 名称不同，
避免在同一 TF 树中产生同名标签的双父节点。

## 6. 真机分阶段入口

以下命令会运动机械臂。必须先核对实际桌面、相机支架、物体尺寸、夹爪宽度、急停和完整路径，
并保持 ROS 控制器速度为 5%。不要在无人监护时执行。

仅到预抓取位置：

```bash
./scripts/run_d435i_apriltag_pregrasp.sh I_CONFIRM_PREGRASP_ONLY
```

D435i 完整抓取和撤退：

```bash
./scripts/run_full_d435i_apriltag_grasp_retreat.sh \
  I_CONFIRM_FULL_VISION_GRASP_RETREAT \
  I_HAVE_CLEARED_THE_WORKSPACE
```

无人机 `tag36h11 ID 0`（边长29 mm）专用分阶段抓取和撤退：

```bash
./scripts/run_full_d435i_drone_grasp_retreat.sh --preflight
./scripts/run_full_d435i_drone_grasp_retreat.sh \
  I_CONFIRM_FULL_DRONE_GRASP_RETREAT \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED
```

该入口固定使用TCP在标签下方45 mm、夹爪张开85 mm/闭合58 mm，并按
100 mm、50 mm、25 mm、最终闭合和持夹撤退执行。50 mm之后标签可能被工具遮挡，
流程会复用已通过30帧稳定度检查的位姿，并检查每一阶段的实际关节反馈。

Gemini 到 D435i 接力抓取：

```bash
./scripts/run_global_to_wrist_apriltag_grasp_retreat.sh \
  I_CONFIRM_GLOBAL_TO_WRIST_FULL_CYCLE \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_OBJECT_STAYS_FIXED_DURING_RUN
```

这些入口包含确认字符串、5% 速度检查、目标位姿稳定度/新鲜度检查、碰撞规划、反馈超时、
夹爪接触范围和阶段报告。软件保护不能替代物理急停或隔离区域。
