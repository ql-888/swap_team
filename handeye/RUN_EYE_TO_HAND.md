# Gemini 336L 眼在手外标定运行指令

默认配置：Gemini 336L 彩色流 640x480@30、顺时针旋转 90 度、ArUco ID 582、
Marker 黑色方框边长 100 mm。相机固定在机械臂外部，Marker 固定在机械臂末端。

## 当前机器

以下三个命令块需要分别在三个终端运行。

### 终端 1：Piper

```bash
cd /home/guoyi/gy_ws/handeye_in/src/piper_ros
bash can_activate.sh can0 1000000

source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye_in/install/setup.bash

ros2 launch piper start_single_piper.launch.py can_port:=can0
```

### 终端 2：Gemini、ArUco 和 rqt 预览

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/OrbbecSDK_ROS2/install/setup.bash
source /home/guoyi/gy_ws/handeye_in/install/setup.bash

ros2 launch handeye_calibration_ros \
  handeye_gemini336l_eye_to_hand.launch.py \
  marker_id:=582 \
  marker_size:=0.10
```

### 终端 3：交互式标定

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye_in/install/setup.bash

ros2 run handeye_calibration_ros handeye_calibration --ros-args \
  -p mode:=eye_to_hand \
  -p min_num:=15 \
  -p piper_topic:=/end_pose \
  -p marker_topic:=/aruco_single/pose \
  -p result_save_path:=/home/guoyi/gy_ws/handeye_in/results/eye_to_hand
```

## 通用指令

首次使用时，将下面两个路径改为实际安装路径并编译：

```bash
HAND_EYE_WS=/absolute/path/to/swap_team/handeye
ORBBEC_WS=/absolute/path/to/OrbbecSDK_ROS2

source /opt/ros/humble/setup.bash
source "$ORBBEC_WS/install/setup.bash"
cd "$HAND_EYE_WS"
colcon build --symlink-install
```

随后分别打开三个终端。每个终端都需要重新设置 `HAND_EYE_WS`；终端 2 还需要
设置 `ORBBEC_WS`。

### 终端 1：Piper

```bash
HAND_EYE_WS=/absolute/path/to/swap_team/handeye

cd "$HAND_EYE_WS/src/piper_ros"
bash can_activate.sh can0 1000000

source /opt/ros/humble/setup.bash
source "$HAND_EYE_WS/install/setup.bash"
ros2 launch piper start_single_piper.launch.py can_port:=can0
```

### 终端 2：Gemini、ArUco 和 rqt 预览

```bash
HAND_EYE_WS=/absolute/path/to/swap_team/handeye
ORBBEC_WS=/absolute/path/to/OrbbecSDK_ROS2

source /opt/ros/humble/setup.bash
source "$ORBBEC_WS/install/setup.bash"
source "$HAND_EYE_WS/install/setup.bash"

ros2 launch handeye_calibration_ros \
  handeye_gemini336l_eye_to_hand.launch.py \
  marker_id:=582 \
  marker_size:=0.10
```

### 终端 3：交互式标定

```bash
HAND_EYE_WS=/absolute/path/to/swap_team/handeye

source /opt/ros/humble/setup.bash
source "$HAND_EYE_WS/install/setup.bash"

ros2 run handeye_calibration_ros handeye_calibration --ros-args \
  -p mode:=eye_to_hand \
  -p min_num:=15 \
  -p piper_topic:=/end_pose \
  -p marker_topic:=/aruco_single/pose \
  -p result_save_path:="$HAND_EYE_WS/results/eye_to_hand"
```

标定终端操作：`Enter` 采集、`d` 删除上一组、`q` 计算并保存、`c` 取消退出。
如画面旋转方向相反，可在终端 2 的 launch 命令末尾增加
`display_rotation:=270`；如不需要 rqt，增加 `start_rqt:=false`。
