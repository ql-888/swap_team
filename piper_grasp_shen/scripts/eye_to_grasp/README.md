# Gemini 336L 全局相机直接抓取

这个目录是独立的 `eye-to-grasp` 流程，不会修改或调用 `scripts/eye_in _grasp/` 的实现。
它使用固定的 Orbbec Gemini 336L 观察无人机 `tag36h11` ID 0（29 mm），采集一次稳定的全局位姿，规划完整的预抓取、抓取和撤退，然后锁定位姿完成动作。机械臂运动后不会再读取 D435i，也不会做视觉修正。

## 启动

先初始化 CAN：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/setup_can.sh can0 1000000
```

终端 1 启动机械臂和 Piper TF：

```bash
./scripts/eye_to_grasp/start_robot_stack.sh
```

终端 2 启动 Gemini 336L、标定 TF 和 29 mm AprilTag：

```bash
./scripts/eye_to_grasp/start_vision_stack.sh
```

如果 Orbbec 工作空间不在默认位置，设置其 `install/setup.bash`：

```bash
ORBBEC_ROS_SETUP=/绝对路径/orbbec_ws/install/setup.bash \
  ./scripts/eye_to_grasp/start_vision_stack.sh
```

终端 3 只读检查：

```bash
./scripts/eye_to_grasp/status.sh
./scripts/eye_to_grasp/run_direct_global_grasp.sh --preflight
```

## 真机抓取

确认无人机、标签和机械臂路径均固定且已清空后，运行：

```bash
./scripts/eye_to_grasp/run_direct_global_grasp.sh \
  I_CONFIRM_EYE_TO_GRASP_DIRECT \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED
```

流程只有四个阶段：一次 Gemini 位姿采集、离线碰撞检查与规划、预抓取后直接下探闭合、持夹撤退。运行日志和锁定位姿保存在 `runtime/eye_to_grasp/`。闭合后只要接触反馈不在 `58--85 mm` 范围内，就拒绝执行撤退。

请保持两个启动终端打开；抓住无人机后不要按 Ctrl-C 或断电。软件检查不能替代急停和现场监护。

## Gemini 相对预观察位 + 原眼在手抓取

确保机械臂、Gemini 和 D435i 三个节点在同一个 `ROS_DOMAIN_ID`（默认 1）运行后执行：

```bash
./scripts/eye_to_grasp/run_global_relative_then_eye_in_grasp.sh \
  I_CONFIRM_GLOBAL_RELATIVE_EYE_IN_GRASP \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED
```

脚本先读取 Gemini 当前 Tag 位姿，套用示教得到的 Tag-to-flange 固定偏移，移动到相对预观察位；随后原样调用 `run_full_d435i_drone_grasp_retreat.sh` 完成 D435i 局部视觉抓取。不会修改 `scripts/eye_in _grasp/`。

预观察规划优先按采集顺序尝试多个原始示教位。只有全部原始示教位都无法通过 IK、工作空间和整条路径碰撞检查时，才会在每个示教位周围尝试小范围 XYZ 平移。候选姿态保持示教值，规划器不会再增加绕法兰轴的旋转候选。

## 采集多个 D435i 预观察候选

采集程序只订阅关节、TF 和双相机图像，不发布控制命令，也不负责显示画面。先在三个终端分别保持机械臂、Gemini 和 D435i 栈运行，并确保它们使用同一个 `ROS_DOMAIN_ID=1`。在第四个终端单独打开 D435i 画面：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
export ROS_DOMAIN_ID=1
source ./scripts/source_ros_env.sh
/usr/bin/python3 ./scripts/eye_to_grasp/view_d435i.py
```

然后在第五个终端运行无窗口采集器：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
export ROS_DOMAIN_ID=1
source ./scripts/source_ros_env.sh
/usr/bin/python3 ./scripts/eye_to_grasp/record_observation_candidates.py \
  --candidates 5
```

D435i 窗口只负责实时显示。在采集终端中，每次把机械臂移动到一个 D435i 能看清无人机标签的预观察位后，按一次回车立即保存一个候选；程序记录 Gemini 看到的无人机标签中心、法兰位姿、六个关节角和双相机彩色图。D435i AprilTag TF 不参与采集或偏移计算。采满设定数量会自动保存；提前完成时可在非采集中输入 `q` 并回车保存退出。结果位于 `runtime/eye_to_grasp/observation_candidates.yaml`。

执行相对预观察流程时，如果旧的 `runtime/eye_to_grasp/dual_camera_observation.yaml` 仍然存在，程序会把旧示教点作为第一个原始候选，再按采集顺序追加新候选。原始记录文件不会被改写。所有原姿态均失败后才生成 XYZ 平移回退，所有候选的 `axial_angles_deg` 保持为 `[0.0]`。

## 开机一键启动并抓取

该入口会先清理本项目残留进程，再启动机械臂、Gemini、D435i 和 TF，等待节点就绪后执行上述完整流程：

```bash
./scripts/eye_to_grasp/boot_and_grasp.sh \
  I_CONFIRM_BOOT_AND_GRASP \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED
```

只清理残留、不启动设备时执行：

```bash
./scripts/eye_to_grasp/cleanup_eye_to_grasp.sh
```
