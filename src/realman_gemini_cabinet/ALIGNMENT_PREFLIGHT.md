# MoveL 对齐预检查（可选模式）

只检查 `knob_camera_servo` 的对齐动作。仍由 `rm_driver` 执行原来的绝对 MoveL，不接入 MoveIt，不选择其他构型，不增加 MoveJ，不检查或改写接近、夹紧、旋转、撤退动作。观察位移动和原来的姿态调整仍在对齐预检查之前执行。

## 使用

重新启动任务节点，沿用原启动文件和其他启动参数，只替换任务配置：

```bash
ros2 launch realman_gemini_cabinet direct_driver_bringup.launch.py \
  task_config:=$(ros2 pkg prefix realman_gemini_cabinet)/share/realman_gemini_cabinet/config/ros2_qr_panel_layout_movel_check.yaml
```

原 `ros2_qr_panel_layout_demo.yaml`、`ros2_qr_named_panel_demo.yaml` 及启动默认值均未修改。原配置不启用预检查。

新配置在原 panel-layout 配置上只增加：

```yaml
alignment_preflight: true
alignment_preflight_step_m: 0.001
alignment_preflight_min_margin_deg: 5.0
alignment_preflight_max_joint_step_deg: 5.0
```

这些是计算检查阈值，不修改控制器限位。5° 是可配置的保守起始余量，不是机械臂厂家保证的安全距离。

## 检查内容

1. 从 `/joint_states` 按 joint1…joint6 名称读取实际角度，从 `tcp_pose_topic` 读取实际位姿。要求新鲜且基本静止的反馈。
2. 检查实际起始关节是否落在算法模型范围内，不对回读关节角裁剪、绕回或替换为其他分支。
3. 核对正运动学与回读位姿（位置误差不超过 3 mm、朝向误差不超过 0.5°）；不一致即拒绝。
4. 按实际对齐轴顺序，以不超过 1 mm 的间隔采样直线，保持实际起始朝向；每个采样点使用上一点关节解作为逆解种子。
5. 检查逆解失败、关节跳变、限位、限位余量，并验证逆解对应的末端位姿。
6. 整个对齐路径先检查一遍，避免 Y 已移动而 Z 才发现不可行；每一轴实际执行前，再从回读状态复查该轴。
7. 通过后，发送与原逻辑相同的 MoveL 命令；失败时不发送本次对齐指令。已经完成的观察、姿态调整或前一对齐轴不会自动撤销。

## 限制和诊断

这是离线采样的运动学预判，不是厂家控制器实际插补器的仿真；不检查速度、加速度、自碰撞或环境碰撞，也不保证实机一定执行成功。启动 SDK 算法时使用 `NULL` 控制器句柄，不创建硬件连接。

当前适配现场 RML63 III 标准版，采用现场 SDK 的出厂模型、法兰坐标系和出厂算法限位；没有读取或覆盖实际控制器限位、DH 标定或工具坐标系。报告中的 `model_source` 明确标注此来源。型号、工具坐标或标定不一致应先解决，不能通过放大检查阈值掩盖。

默认 SDK 路径由 rm_driver 的安装前缀推导为工作空间 `src/ros2_rm_robot/rm_driver/lib/libapi_cpp.so`。其他目录可在技能参数中指定 `alignment_preflight_sdk_library` 绝对路径。仅使用本工作空间已验证的 SDK ABI。

状态话题 `/rg_cabinet/status` 发布 `event=alignment_preflight`。失败的任务服务响应增加 `alignment_preflight` 字段，包含失败轴、采样位置、限制关节、最小余量和 SDK 返回码等信息。

常见原因：

- `MARGIN_TOO_SMALL`：预测路径上的限位余量不足。
- `JOINT_BRANCH_JUMP`：相邻采样点出现大幅关节跳变。
- `START_OUTSIDE_MODEL_LIMITS`：实际角度已超出出厂算法模型范围，需要核对现场限位和当前状态。
- `MODEL_OR_TCP_FRAME_MISMATCH`：正运动学与实际回读不匹配。
- `PREFLIGHT_UNAVAILABLE`：SDK 缺失、无新鲜静止反馈等；拒绝放行，不绕过检查。

## 已做验证

对上次失败路径的离线复算：原起始分支在约 13.97 mm 处预测 J4 余量不足 5°；仅在诊断回放中将余量设为 0° 后，约 16.97 mm 处仍因关节分支跳变被拒绝。另一个用于测试的连续分支通过 Y/Z 共 202 个采样点；程序不会自动切换到该分支。上述回放不发送任何运动指令。
