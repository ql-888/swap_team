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

## 眼在手上：D435i

只连接 D435i 标定相机，然后运行：

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch handeye_calibration_ros handeye_eye_in_hand.launch.py
```

该入口使用 `eye_in_hand` 模式，并自动启动 D435i、Piper、法兰位姿、ChArUco 检测、标定会话和 rqt 采集面板。标定结果保存到：

```text
/home/guoyi/gy_ws/handeye/results
```

## 眼在手外：Gemini 336L

只连接 Gemini 336L 标定相机，然后运行：

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/OrbbecSDK_ROS2/install/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch handeye_calibration_ros handeye_eye_to_hand.launch.py
```

该入口使用 `eye_to_hand` 模式，并自动启动 Gemini 336L、Piper、ChArUco 检测、标定会话和 rqt 采集面板。标定结果保存到：

```text
/home/guoyi/gy_ws/handeye/results
```

## 自动识别模式

只连接一台标定相机时，也可以让程序自动选择模式：D435i 对应眼在手上，Gemini 336L 对应眼在手外。

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/OrbbecSDK_ROS2/install/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

ros2 launch handeye_calibration_ros handeye_auto.launch.py
```

## rqt 采集面板

上述三个 launch 入口都会自动打开 rqt 面板。需要单独重新打开面板时执行：

```bash
source /opt/ros/humble/setup.bash
source /home/guoyi/gy_ws/handeye/install/setup.bash

rqt --force-discover --standalone handeye_calibration_ros.handeye_rqt.HandeyeRqtPlugin
```

面板操作：`Space` 采集、`D` 删除上一组、`Q` 计算并保存、`Esc` 退出。至少采集 15 组，并同时改变机械臂的位置和观察角度。

## Gemini 336L 与 100 mm ArUco Marker 独立流程

下面保留 ID 582、Marker 黑色外框边长 100 mm 的独立眼在手外流程。三个命令块分别在三个终端运行。

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
