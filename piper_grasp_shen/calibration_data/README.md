# 相机标定数据

本目录只保存当前运行使用的原始标定报告：

- `d435i_eye_in_hand.json`：D435i 腕部相机，`flange_link_T_camera`。
- `gemini336l_eye_to_hand.json`：Gemini 336L 固定相机，`base_link_T_camera`。

`config/handeye_d435i_wrist_end_pose.yaml` 和
`config/handeye_orbbec_fixed.yaml` 是运行时使用的规范化外参。JSON 用于溯源和
一致性测试，不会在启动抓取流程时重新计算标定。

更换相机安装位置或机械结构后，旧外参立即失效；应重新标定并同时更新 JSON、
运行 YAML 和一致性测试。长度单位为米，四元数顺序为 `x, y, z, w`。
