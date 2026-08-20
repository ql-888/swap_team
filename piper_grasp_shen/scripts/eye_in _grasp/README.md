# D435i 眼在手无人机抓取（3个终端）

目录名包含空格，命令必须写成 `./scripts/'eye_in _grasp'/...`。

## 每次开机

先初始化CAN：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/setup_can.sh can0 1000000
```

终端1启动机械臂控制和Piper X反馈TF：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/'eye_in _grasp'/start_robot_stack.sh
```

终端2启动D435i、手眼TF和29 mm AprilTag检测：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/'eye_in _grasp'/start_vision_stack.sh
```

终端3先做只读状态检查：

```bash
cd /home/guoyi/gy_ws/piper_grasp_shen
./scripts/'eye_in _grasp'/status.sh
```

只有显示 `EYE_IN_GRASP_STATUS_READY` 才能运行真实抓取：

```bash
./scripts/run_full_d435i_drone_grasp_retreat.sh \
  I_CONFIRM_FULL_DRONE_GRASP_RETREAT \
  I_HAVE_CLEARED_THE_WORKSPACE \
  I_CONFIRM_DRONE_AND_TAG_FIXED
```

终端1和终端2必须保持打开。无人机仍被夹持时，禁止关闭终端1、按Ctrl-C或断电。
详细日志保存在 `runtime/eye_in_grasp/logs/`。
