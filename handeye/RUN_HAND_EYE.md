# Handeye 眼在手上与眼在手外运行指令

本工作空间同时支持：

- `eye_in_hand`：D435i 安装在机械臂末端。
- `eye_to_hand`：Gemini 336L 固定在机械臂外部。

本机工作空间为 `/home/guoyi/gy_ws/handeye`。下面均为本机实际路径，命令块需要在新终端中完整执行。

## 首次编译

```bash
cd /home/guoyi/gy_ws/handeye
source /opt/ros/humble/setup.bash
source /home/guoyi/OrbbecSDK_ROS2/install/setup.bash
colcon build --symlink-install
```

## CAN 接口未启动时

电脑重启或 USB-CAN 重新连接后，如果程序提示 `can0 is not UP`，先执行：

```bash
cd /home/guoyi/gy_ws/handeye/src/piper_ros
bash can_activate.sh can0 1000000
```

## 眼在手上：D435i 与 100 mm ArUco Marker

使用 ID 582、Marker 黑色外框边长 100 mm。下面五个命令块必须分别在五个终端中运行。

### 终端 1：激活 CAN 并启动 Piper

```bash
cd /home/guoyi/gy_ws/handeye/src/piper_ros
bash can_activate.sh can0 1000000

source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=false
```

### 终端 2：启动 D435i 彩色相机

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true \
  -p enable_depth:=false \
  -p enable_infra1:=false \
  -p enable_infra2:=false \
  -p enable_gyro:=false \
  -p enable_accel:=false \
  -p rgb_camera.color_profile:=640x480x30 \
  -r /camera/camera/color/image_raw:=/stereo/left/image_rect_color \
  -r /camera/camera/color/camera_info:=/stereo/left/camera_info
```

### 终端 3：启动 ArUco 检测

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch aruco_ros single.launch.py \
  eye:=left \
  marker_id:=582 \
  marker_size:=0.10
```

### 终端 4：打开 rqt 视频

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 run rqt_image_view rqt_image_view
```

在 rqt 图像话题中选择 `/aruco_single/result`。

### 终端 5：启动眼在手上标定程序

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 run handeye_calibration_ros handeye_calibration --ros-args \
  -p mode:=eye_in_hand \
  -p min_num:=15 \
  -p piper_topic:=/end_pose \
  -p marker_topic:=/aruco_single/pose \
  -p result_save_path:=/home/guoyi/gy_ws/handeye/results/eye_in_hand
```

## 眼在手外：Gemini 336L 与 100 mm ArUco Marker

使用 ID 582、Marker 黑色外框边长 100 mm。下面三个命令块必须分别在三个终端中运行。

### 终端 1：Piper

```bash
cd /home/guoyi/gy_ws/handeye/src/piper_ros
bash can_activate.sh can0 1000000

source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch piper start_single_piper.launch.py can_port:=can0
```

### 终端 2：Gemini、ArUco 和 rqt 视频

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/OrbbecSDK_ROS2/install/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch handeye_calibration_ros \
  handeye_gemini336l_eye_to_hand.launch.py \
  marker_id:=582 \
  marker_size:=0.10
```

### 终端 3：独立标定程序

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 run handeye_calibration_ros handeye_calibration --ros-args \
  -p mode:=eye_to_hand \
  -p min_num:=15 \
  -p piper_topic:=/end_pose \
  -p marker_topic:=/aruco_single/pose \
  -p result_save_path:=/home/guoyi/gy_ws/handeye/results/eye_to_hand
```

独立标定终端操作：`Enter` 采集、`d` 删除上一组、`q` 计算并保存、`c` 取消退出。

## 可选：一键 ChArUco 标定入口

下面的一键入口使用 `config/charuco.yaml` 中的 ChArUco 标定板配置，不是上面的 ID 582、100 mm 单 ArUco Marker 流程，不能混用标定板。

眼在手上只连接 D435i：

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch handeye_calibration_ros handeye_eye_in_hand.launch.py
```

眼在手外只连接 Gemini 336L：

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/OrbbecSDK_ROS2/install/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch handeye_calibration_ros handeye_eye_to_hand.launch.py
```

自动识别相机和标定模式：

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/OrbbecSDK_ROS2/install/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch handeye_calibration_ros handeye_auto.launch.py
```
