# 基于ROS2的手眼标定程序包

## 一键标定（正式入口）

连接 Piper 机械臂和**一台**标定相机后，只运行：

```bash
source /opt/ros/humble/setup.bash
source ~/gy_ws/handeye/install/setup.bash
ros2 launch handeye_calibration_ros handeye_auto.launch.py
```

程序自动完成以下操作，不需要选择相机或标定模式：

- 检测到 D435i：自动启动 RealSense，并执行 `eye_in_hand`
  - 使用 `/joint_states_feedback` 和 Piper URDF 正运动学计算 `base_link -> link6`
  - 标定输入为法兰 `link6` 位姿，不读取控制器 TCP 末端位姿
- 检测到 Gemini 336L：自动启动 Orbbec，并执行 `eye_to_hand`
- 启动 Piper 驱动，但不会自动使能机械臂
- 检查 USB 3.x、CAN、彩色图像、CameraInfo、ChArUco 和末端位姿
- 自动打开 rqt 采集面板，预览和采样控制与检测线程分离
- 用五种 OpenCV 手眼算法交叉计算，只保存通过质量门槛的结果

为防止标定类型用错，未连接目标相机、同时连接两台目标相机、相机落在
USB 2.0、USB-CAN 未连接、`can0` 未生成、未启用或波特率不是 1 Mbps
时，程序会拒绝启动并说明准确原因。

Piper 的 candleLight USB-CAN 转接器在 `lsusb` 中应显示为 `1d50:606f`。
转接器每次重新连接或电脑重启后，如果 `can0` 尚未启用，launch 会请求一次
管理员授权，并自动按 1 Mbps 启用。如果系统授权失败，可手动执行：

```bash
cd ~/gy_ws/handeye/src/piper_ros
bash can_activate.sh can0 1000000
```

该脚本需要输入管理员密码来配置 Linux 网络接口。正常情况下，标定只需要
上面的一条 `ros2 launch` 命令。

采集面板操作：

- `Space`：采集一组稳定姿态
- `D`：删除上一组
- `Q`：质量检查、计算并保存
- `Esc`：退出

也可以单独打开面板：

```bash
source /opt/ros/humble/setup.bash
source ~/gy_ws/handeye/install/setup.bash
rqt --force-discover --standalone handeye_calibration_ros.handeye_rqt.HandeyeRqtPlugin
```

面板只有在相机在线、标定板连续稳定、机械臂静止、时间配对误差不超过
200 ms 且姿态没有接近任何已采样点时才启用 `Capture sample`。程序会按
ChArUco 消息时间戳寻找最近的 Piper `/end_pose_stamped`，不会直接使用旧的
最后一帧位姿。

面板实时显示当前姿态的采集质量评分：

- `Board 0-40`：角点完整度、板面在图像中的面积和清晰度
- `Novelty 0-40`：当前机械臂姿态与所有已采集姿态的差异
- `Coverage 0-20`：是否扩展 XYZ 位置和三个旋转方向的覆盖范围
- `GOOD`：80-100，优先采集；`USABLE`：60-79，可以采集；`LOW`：建议调整

评分沿用旧 D435i 采集程序的 40/40/20 公式。眼在手上使用法兰 `link6`
位姿计算新颖度和覆盖度，不受控制器 TCP 设置影响。评分用于选择更有价值的
采集姿态，原有稳定、同步和重复姿态检查仍是采集按钮的硬性条件。

至少采集 15 组。每组之间需要同时改变位置和观察角度，尽量覆盖机械臂的
可用范围；程序会拒绝重复姿态、运动中的数据和不稳定的标定板检测。

只有质量检查通过才会在 `~/gy_ws/handeye/results` 同时生成 JSON 和 YAML：

- D435i：结果为 `link6_T_camera`，即法兰到相机光学坐标系
- Gemini 336L：结果为 `base_T_camera`

### 眼在手上：法兰位姿入口

D435i 和力传感器安装完成后，只连接 D435i，运行：

```bash
source /opt/ros/humble/setup.bash
source ~/OrbbecSDK_ROS2/install/setup.bash
source ~/gy_ws/handeye/install/setup.bash
ros2 launch handeye_calibration_ros handeye_eye_in_hand.launch.py
```

该入口自动启动 `/flange_pose_stamped`。位姿由六个反馈关节角和 Piper URDF
计算，坐标方向为 `base_link -> link6`，不会使用 `/end_pose_stamped`，因此
控制器中 TCP 是 145 mm、198 mm 或其他值都不会改变本次标定输入。标定后
相机和法兰之间的机械安装关系不能改变；力传感器或相机重新拆装后需要重标。

当前 ChArUco 配置已经按提供的生成器参数写入：6×9、`DICT_4X4_50`、
起始 ID 0、方格 30 mm、marker 21.6 mm。

## ChArUco 配置

新增的 `charuco_detector` 使用相机发布的彩色图像和 `CameraInfo` 计算
ChArUco 标定板相对于相机光学坐标系的位姿，并发布：

- `/charuco/pose`：标定板位姿，类型为 `geometry_msgs/PoseStamped`
- `/charuco/result`：画有 marker、ChArUco 角点和坐标轴的结果图像
- `charuco_board`：可选 TF 子坐标系

标定板参数在 `config/charuco.yaml` 中配置。长度单位必须是米：

|参数|说明|
|---|---|
|`columns`、`rows`|棋盘格列数和行数，不是内部角点数|
|`dictionary`|生成标定板时选择的 ArUco 字典|
|`marker_id_offset`|第一个 marker 的 ID|
|`square_length`|打印后实测的棋盘方格边长|
|`marker_length`|打印后实测的黑色 ArUco marker 外框边长|
|`min_charuco_corners`|发布位姿至少需要识别的 ChArUco 角点数|

默认值对应所提供的标定板：6 列、9 行、`DICT_4X4_50`、起始 ID 0、
方格 30 mm、marker 21.6 mm。修改配置后需要重新编译本包。

## 构建

```bash
cd ~/gy_ws/handeye
source /opt/ros/humble/setup.bash
source ~/OrbbecSDK_ROS2/install/setup.bash
colcon build --symlink-install --packages-select handeye_calibration_ros
```

`OrbbecSDK_ROS2` 可以放在本工作区外；当前工作区的安装环境已经记录了它的
overlay。若重新创建整个工作区，构建前继续按上面顺序 source 即可。
