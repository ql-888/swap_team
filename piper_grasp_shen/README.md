# Piper X 精确抓取工程

> 本副本已针对 Piper X 适配：使用 `agx_arm_ros` 官方 Piper X 带夹爪
> URDF/网格，并按 Piper X 的 joint4/joint5 限位构建 Pinocchio 模型。
> 默认 TCP 已设为当前实物总成的标定结果：相对 `flange_link` 的
> `[0, 0, 0.198] m / [0, 0, 48.2] degree`。其中 Z 来自 TCP 枢轴标定，
> yaw 来自力传感器与夹爪重新安装后的三组水平开合方向观测。

这是一个面向 Ubuntu 22.04 和 AgileX Piper X 的完整抓取基础工程。它使用 Piper SDK 负责 CAN 通信，Pinocchio 负责机器人模型、正运动学和碰撞模型，Pink 负责带关节限位的多初值逆运动学，OpenCV ArUco 提供一个可替换的 6D 目标检测入口。

工程默认只做离线计算。真实运动必须同时提供 `--execute` 和精确确认字符串 `I_HAVE_CLEARED_THE_WORKSPACE`，否则不会使能机械臂。离线构建和验收命令不会打开 `can0`，也不会发送真实运动命令。

## 已实现能力

- 项目内独立 Conda 环境，不污染 ROS 或系统 Python。
- Piper ROS 官方 URDF、网格和 MoveIt SRDF 随项目保存。
- 六轴 Pinocchio 模型、实测 TCP、FK 和 Pink 多初值全位姿 IK。
- 多个绕抓取轴的候选姿态，自动跳过不可达或碰撞候选。
- 当前位姿到预抓取、抓取、撤离的整段关节插值碰撞检查。
- 关节限位、工作空间、桌面/障碍物、自碰撞、NaN 和反馈超时保护。
- TCP 枢轴标定，以及已导入的 D435i 腕部相机和 Gemini 336L 固定相机手眼结果。
- ArUco 图像检测、统一目标位姿 YAML、物体抓取配方和运行报告。
- 只读采集 TCP/手眼样本；采样命令不会使能或移动机械臂。

## 目录

```text
piper_grasp/
  assets/                 Piper URDF、STL 和 SRDF
  calibration_data/       当前 D435i 与 Gemini 336L 原始标定报告
  config/                 相机、场景、位姿、抓取和运行标定配置
  docs/                   目录结构与本机运行指引
  runtime/                自动生成的位姿、规划和执行历史
  scripts/                环境、CAN、运行和测试脚本
  src/piper_pink/          规划、标定、视觉、碰撞和 SDK 桥接代码
  tests/                   离线单元与集成测试
```

## 1. 安装环境

当前项目目录为 `/home/guoyi/gy_ws/piper_grasp_shen`。环境已从已验证副本通过
Conda 克隆到项目内 `.conda`；首次或代码更新后运行：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/setup_env.sh
./scripts/run.sh doctor --handeye config/handeye_orbbec_fixed.yaml
./scripts/test.sh
./scripts/verify_offline.sh
```

环境安装在项目自己的 `.conda` 目录。以后不需要激活 Conda，统一通过
`scripts/run.sh` 调用，脚本会清除 ROS 的 `PYTHONPATH`，避免 ROS Python 包污染
Pinocchio 环境。目录说明见 `docs/STRUCTURE.md`，设备启动顺序见
`docs/DEVICE_SETUP.md`。

已验证版本：Python 3.10、Pinocchio 4.1.0、Pink 4.3.0、OpenCV 5.0.0、Piper SDK 0.6.2。

## 2. CAN 驱动和当前 USB 情况

Piper SDK 的默认接口是 Linux SocketCAN `can0`，Piper 总线通常配置为 1 Mbit/s。插入兼容 `gs_usb` 的 USB-CAN 适配器后运行：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/setup_can.sh can0 1000000
ip -details -statistics link show can0
./scripts/run.sh doctor --require-can
./scripts/run.sh read-arm --can can0
```

此前系统日志只识别到以下设备：

- `2341:0043` Arduino，注册为 `ttyACM0`。
- `1a86:7523` CH341 USB 串口，曾注册为 `ttyUSB0`，随后被 BRLTTY 抢占。
- GenesysLogic USB 2/3 Hub。

这些记录里没有 USB-CAN 设备，所以加载 `gs_usb` 后仍不会出现 `can0`。Type-C 扩展坞本身没有问题，但必须接入真正支持 SocketCAN/`gs_usb` 的 USB-CAN 适配器。不要把普通 Arduino 或 CH341 USB 串口直接当成 Piper CAN 驱动；除非该设备明确烧录了 SLCAN 固件，并按 SLCAN 方式配置。

## 3. 离线验收

先确认模型和依赖：

```bash
./scripts/run.sh info
./scripts/run.sh doctor --handeye config/handeye_orbbec_fixed.yaml
```

运行示例抓取规划并保存可追踪报告：

```bash
./scripts/run.sh plan-grasp \
  --object-pose config/object_pose.example.yaml \
  --recipe config/grasp_recipe.example.yaml \
  --report logs/example_plan.yaml
```

示例目标已验证能够产生预抓取、抓取和撤离三段解，所有插值路点都经过自碰撞和场景碰撞检查。`plan-grasp` 不会打开 CAN。

## 4. 坐标和数据接口

所有长度在算法内部使用米，角度在算法内部使用弧度。Piper SDK 边界会将关节角转换成协议的 `0.001 degree`；夹爪开度转换成协议的 `0.001 mm`。

目标检测器需要生成与 `config/object_pose.example.yaml` 相同的文件：

```yaml
frame_id: camera_color_optical_frame
object_id: demo_block
confidence: 0.95
timestamp_s: 1786240000.0
frame_from_object: [[...], [...], [...], [0, 0, 0, 1]]
```

`frame_from_object` 是 `frame_id` 到物体坐标系的齐次变换。相机坐标系输入必须和手眼标定的 `camera_frame` 完全一致。真实抓取默认拒绝超过 2 秒或没有时间戳的目标位姿。

ArUco 示例：

```bash
./scripts/run.sh detect-aruco \
  --image /absolute/path/frame.png \
  --intrinsics config/camera_intrinsics.yaml \
  --marker-id 0 --marker-size-m 0.040 \
  --object-id demo_block --output /tmp/object_pose.yaml
```

`config/grasp_recipe.example.yaml` 中的 `object_from_tcp` 是物体坐标系下期望 TCP 位姿。每种物体、夹持方向和夹爪宽度应使用独立配方。

当前 D435i AprilTag 参数为 `tag36h11`、ID `0`、黑色编码方形边长
`18 mm`。D435i 驱动运行后，在另一个终端启动官方 AprilTag ROS 节点：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/run_apriltag_d435i.sh
```

检测结果发布在 `/perception/d435i/detections`，标签 TF 名为
`apriltag_0`。配置文件是 `config/apriltag_d435i.yaml`。

机械臂状态 TF 在线后，可在独立终端发布已标定的法兰到 D435i 静态变换：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/publish_d435i_handeye_tf.sh
```

标定结果本身是 `flange_link -> d435i_color_optical_frame`。由于 RealSense 驱动
已经发布 `d435i_link -> d435i_color_frame -> d435i_color_optical_frame`，脚本会
先把标定结果等价换算为 `flange_link -> d435i_link`，从 D435i 根坐标连接两棵
TF 树，避免给彩色光学坐标设置第二个父节点。随后用
`tf2_echo piper_x/base_link apriltag_0` 检查标签在机械臂底座坐标系中的位姿。

Gemini 336L 使用独立的标签坐标系 `gemini_apriltag_0`，避免与 D435i
的 `apriltag_0` 发生 TF 双父节点冲突：

```bash
./scripts/publish_gemini336l_handeye_tf.sh
./scripts/run_apriltag_gemini336l.sh
ros2 run tf2_ros tf2_echo piper_x/base_link gemini_apriltag_0
```

队友标定结果为 `base_link -> gemini336l_color_optical_frame`。因为 Orbbec
驱动已发布 `gemini336l_link -> gemini336l_color_frame ->
gemini336l_color_optical_frame`，脚本会等价换算并发布
`piper_x/base_link -> gemini336l_link`。

### D435i 顶部 AprilTag 两指抓取规划

`config/grasp_recipe_apriltag_top.yaml` 定义了当前抓取几何：

- AprilTag `+X` 作为左右方向，对应 Piper X TCP `+X` 的两指闭合方向；
- TCP 沿标签 `-Z` 进入物体 `25 mm`；
- 接近轴相对标签法向倾斜 `30°`，并绕标签 `X` 轴倾斜，因此不改变左右夹取方向；
- 预抓取和后退距离均为 `100 mm`。

在机械臂、D435i、D435i 手眼 TF 和 AprilTag 检测节点均运行时，
使用下列命令采集 30 帧并只做离线路径规划：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/plan_d435i_apriltag_grasp.sh
```

脚本会生成 `runtime/d435i_apriltag_object_pose.yaml` 和
`runtime/d435i_apriltag_grasp_plan.yaml`。该命令不打开 CAN、不使能、不移动
机械臂、不操作夹爪。

当前配方的 `execution_ready: false`是硬件执行锁。在实际执行前还必须测量
物体宽度并设置 `gripper_closed_m`，核对标签的 `25 mm` 抓取深度，
以及用实际桌面、物体、相机和支架尺寸补齐 `config/scene.yaml`。

首次真机测试使用专用的预抓取入口，速度限制为 `5%`，夹爪打开到
`70 mm`，并且只运动到预抓取点：

```bash
./scripts/run_d435i_apriltag_pregrasp.sh I_CONFIRM_PREGRASP_ONLY
```

该入口不会下降到抓取点、不会闭合夹爪、不会执行后退段。
由于本机 IK 和全路径碰撞检查实测需要约 `6-8 s`，该预抓取入口允许
从采集到开始动作最多 `15 s`；运行期间标签和物体必须刚性固定不动。
该脚本只发布 TF，不连接 CAN，也不发送机械臂控制命令。

### 无人机抓取后运送到 Standard52h13 目标上方

目标标签为 `TagStandard52h13` ID 0，实测最外侧虚线框边长 `37 mm`。该家族
`width_at_border=6`、`total_width=10`，所以 AprilTag 位姿计算尺寸为
`37 × 6 / 10 = 22.2 mm`。无人机使用
`tag36h11` ID 0、黑框边长 `29 mm`。无人机抓取约束为 TCP 沿标签 `-Z`
下探 `45 mm`、夹爪张开 `85 mm`、闭合并持续保持 `58 mm`。

只执行腕部相机识别、分段抓取和撤退，不执行运输时，先启动无人机专用
D435i AprilTag 节点，然后运行：

```bash
./scripts/run_apriltag_d435i_drone.sh

./scripts/run_full_d435i_drone_grasp_retreat.sh \
  I_CONFIRM_FULL_DRONE_GRASP_RETREAT \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED
```

可先用 `./scripts/run_full_d435i_drone_grasp_retreat.sh --preflight` 只读核对
29 mm 标签、45 mm 下探、85/58 mm 夹爪参数和六个执行阶段。预检不会连接控制器或运动。

Gemini 图像和最新眼在手外 TF 已运行后，另开终端启动目标标签检测：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/run_apriltag_gemini336l_standard52h13.sh
```

确认无人机、两个标签在整个流程中固定，且抓取与搬运路径均已清空后运行：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/run_full_drone_grasp_transport_standard52.sh \
  I_CONFIRM_FULL_DRONE_GRASP_TRANSPORT \
  I_CONFIRM_OBJECT_AND_TARGET_TAGS_FIXED \
  I_HAVE_CLEARED_GRASP_AND_TRANSPORT_PATH
```

流程依次执行腕部相机抓取、完整撤离，然后由 Gemini 重新采集 20 帧目标位置，
保持 `58 mm` 夹紧命令，并用和无人机抓取相同的方向约束，使 TCP `+X` 两指开合线
与目标标签 `+X` 左右方向对齐，将 TCP 移动至目标标签 `base_link +Z` 方向上方
`150 mm`。流程不会下降或释放无人机。当前碰撞模型不含
无人机外形，因此操作员必须为无人机机身和旋翼额外留出完整路径空间。

### 完整抓取并放置到平台

已经真机分步验证的完整交互入口位于
`scripts/to_platform/run_grasp_place_complete.sh`。它从
`eye_to_grasp/boot_and_grasp.sh` 开始，依次完成抓取、安全抬升、平台码定位、
平台上方对准、下降 120 mm、夹爪增开 10 mm，以及保持 XY/姿态上撤 120 mm。
程序在抓取后、平台上方和松爪前分别暂停，必须由现场操作员输入对应确认词才会继续。
具体命令、安全条件和确认词见 `scripts/to_platform/README.md`。

## 5. 当前 TCP

当前默认值是装好力传感器并重新安装夹爪后得到的实测 TCP：相对法兰盘
`X=0, Y=0, Z=198 mm, RX=0, RY=0, RZ=48.2 degree`。其中位置来自枢轴标定，
RZ 来自三组实际水平开合方向观测（49.672°、47.519°、47.392°）的综合估计。
只要工具总成没有改变，无需重复标定。更换力传感器、转接板、夹爪或工具尖端后，
当前 TCP 和相机外参都应视为失效；请在独立标定工程中重新计算并更新
`config/tcp_piper_x_measured.yaml` 与相机外参。

## 6. 手眼标定

旧的单位矩阵占位配置和“从零采样”主流程已经移除，现采用当前交付的标定结果。
两份原始 JSON 保存在 `calibration_data/`，说明见
`calibration_data/README.md`；运行时读取 `config/` 下的规范化外参 YAML。

### Gemini 336L：固定全局相机

运行配置是 `config/handeye_orbbec_fixed.yaml`，内容为 2026-08-18 最新 16 组样本求得的
`base_T_camera`。原结果坐标名 `base` 已确认就是本机的 `base_link`，即机械臂
底座物理原点。可离线检查：

```bash
./scripts/run.sh validate-handeye \
  --calibration config/handeye_orbbec_fixed.yaml
./scripts/run.sh doctor --handeye config/handeye_orbbec_fixed.yaml
```

原结果标记 `quality_passed: true`；实测残差为平移 RMS 7.716 mm、最大
22.489 mm，旋转 RMS 1.488°、最大 3.292°。

### D435i：腕部局部相机

运行配置是 `config/handeye_d435i_wrist_end_pose.yaml`，来自 30 组样本，变换为
`flange_link_T_camera`。实测残差为平移 RMS 6.226 mm、最大 12.199 mm，旋转
RMS 0.673°、最大 1.467°。

这次采集读取旧 ROS 话题 `/end_pose`。发布代码直接读取 Piper SDK 的
`GetArmEndPoseMsgs()`；SDK 自带 TCP 示例把它称为 `J6 Pos`，并在此基础上另加
工具偏移才得到 `TCP Pos`。同时 Piper X URDF 中 `link6 -> flange_link` 是单位
变换，所以这里将 `/end_pose` 正式映射为 `flange_link`，不会错误叠加本项目
198 mm 的 TCP。运行时仍会按配置中的 `parent_frame` 查找模型坐标系。

原始结果中的相机名 `stereo_gazebo_left_camera_optical_frame` 是旧 ArUco launch
硬编码的示例标签，并非额外的物理 TF。当前 D435i 的彩色图像和 CameraInfo 已
实测统一发布 `d435i_color_optical_frame`，所以运行配置采用这个真实名称；标定
矩阵本身保持不变。

两组结果都已完成求解，但都高于旧文档的 2 mm / 0.5° 参考阈值。低残差也不
等于绝对准确，闭环控制前仍应在未参与标定的检查点验证相机到 TCP 的方向、尺度
和重复误差。相机、支架、力传感器或工具总成一旦拆装，相关结果必须重新验证或
重新标定。

## 7. 场景和抓取规划

在 `config/scene.yaml` 中用 `base_link` 坐标系的长方体描述桌面、相机支架、料箱和围栏。真实运行前必须按现场测量更新尺寸和位置。规划器会：

1. 围绕抓取轴生成多个姿态。
2. 对每个姿态执行多初值 Pink IK。
3. 检查工作空间、关节限位和目标误差。
4. 检查当前位姿到预抓取、抓取和撤离的所有离散路径点。
5. 从全部可行候选中选择综合代价最低的解。

当前局部规划器验证关节空间直线，不会绕过障碍物。如果直线路径被阻挡，它会安全失败。需要在复杂货架或密集障碍物中绕行时，应在本工程模型和标定基础上接入 MoveIt 2/OMPL，再把生成轨迹交给相同的安全执行层复检。

## 8. 首次真实抓取

真实机械臂测试必须有人在急停旁监护，先移除负载和障碍物，并把速度保持在 10% 或更低。顺序如下：

```bash
./scripts/run.sh doctor --require-can --handeye config/handeye_orbbec_fixed.yaml
./scripts/run.sh read-arm --can can0
./scripts/run.sh grasp --can can0 \
  --object-pose /tmp/object_pose.yaml \
  --recipe config/grasp_recipe.yaml \
  --handeye config/handeye_orbbec_fixed.yaml \
  --scene config/scene.yaml \
  --report logs/first_grasp.yaml
```

上面的命令只连接、读取并规划，不会使能或运动。确认计划、现场、急停和目标位姿都正确后，才在命令末尾显式增加：

```text
--execute --confirmation I_HAVE_CLEARED_THE_WORKSPACE
```

执行期间任何反馈频率下降、时间戳停止、关节步长超限、碰撞或总超时都会停止继续发送目标。软件停止不是功能安全急停，物理急停和隔离区域仍然必需。

## 9. 精度结论

TCP 标定和手眼标定是精确抓取的必要条件，但不是充分条件。最终误差还取决于：

- 相机内参、畸变、深度/标记检测和时间同步。
- URDF/DH 参数是否与机械臂硬件版本一致。
- 夹爪指尖几何、物体尺寸、夹持配方和夹爪回差。
- 基座安装刚度、关节回差、负载变形和温漂。
- 场景坐标测量、碰撞模型近似和目标遮挡。
- 抓取前最后一次视觉复测或闭环视觉伺服。

本工程解决的是原 SDK 单点逆解盲区较多、缺少多候选和碰撞检查的问题，但不能在没有真实标定和重复误差数据时承诺毫米级精度。建议在 10 到 20 个独立检查点记录位置/姿态误差，按 P95 误差决定抓取容差；要求更高时增加抓取前的视觉闭环修正。

## 上游项目

- Piper SDK: <https://github.com/agilexrobotics/piper_sdk>
- Piper ROS: <https://github.com/agilexrobotics/piper_ros>
- Pinocchio: <https://stack-of-tasks.github.io/pinocchio/>
- Pink: <https://github.com/stephane-caron/pink>
