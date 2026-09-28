# 可选的面板布局定位模式

原有 `gray_plate`、`square_frame`、颜色块路径均保留。原配置 `ros2_qr_named_panel_demo.yaml` 和 launch 默认配置不变。

新增 `locator: panel_layout`。只在明确选择该模式时加载 SIFT/OpenCV 定位器。新模式检测失败直接报错，不回退到原来的黑色色块路径。

## 选择方式

新增完整任务配置：`config/ros2_qr_panel_layout_demo.yaml`。这份配置从原配置复制，只修改黑色旋钮的检测模式，增加模板路径和目标标识；运动参数与原配置一致。

```yaml
vision:
  targets:
    middle_black_knob:
      locator: panel_layout
      panel_layout_template: panel_layout/template.json
      panel_layout_target_id: middle_black_knob
```

`panel_layout_template` 相对当前 `task_config` 所在目录解析，也支持绝对路径。模板内 `reference_image` 相对模板 JSON 所在目录解析，源码目录和安装目录都可用。参考图与模板随包安装。每次任务开始清空模板缓存，同一次任务的多帧检测复用已加载的特征。

在原有启动命令上仅增加或替换：

```bash
task_config:=/home/sunrise/wheel_S200_src/install/realman_gemini_cabinet/share/realman_gemini_cabinet/config/ros2_qr_panel_layout_demo.yaml
```

切回原模式时使用：

```bash
task_config:=/home/sunrise/wheel_S200_src/install/realman_gemini_cabinet/share/realman_gemini_cabinet/config/ros2_qr_named_panel_demo.yaml
```

本变更不自动切换运行节点，也不触发任务或机械臂运动。新配置与代码需要由重新启动的节点加载。

## 输出与边界

`target.details.locator` 为 `panel_layout`，`panel_layout` 字段保存全局匹配质量、局部检测结果及拒绝原因。开启 `save_debug_images` 时，在 `vision_debug_dir` 写入 `*_panel_layout.png`：黄色预测区域、绿色底板、紫色轴帽中心。

新模式使用经过历史图验证的同一二维算法：SIFT/RANSAC 面板匹配、局部四边形检测、浅色轴帽检测。不采用黑色色块重心替代转轴中心。

深度仍复用原有底板周围采样及中心采样；TF、TCP 对齐、深度范围、工作空间约束和运动流程也沿用现有实现。此次没有新增深度平面拟合、时间同步、多帧身份跟踪或更改进给距离。二维规则通过不等于三维抓取精度已验证。

当前模板只针对现场这块柜体及其具有浅色中心轴帽的旋钮。大视角变化或遮挡时可能拒绝，需要重新观察；不能把新模式当成所有柜体通用的检测器。

## 验证

```bash
source /opt/ros/humble/setup.bash
cd /home/sunrise/wheel_S200_src/src/realman_gemini_cabinet
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:$PYTHONPATH python3 -m pytest -q tests/test_panel_layout_mode.py tests/test_vision_target_resolver.py tests/test_image_tools.py tests/test_launch_defaults.py
```

设备的用户级 anyio pytest 插件与系统 pytest 不兼容，以上命令禁用自动加载第三方测试插件；本组测试不依赖这些插件。

新测试覆盖模式选择、相对模板路径、目标 ID、深度/工作空间拒绝、失败不回退、轴帽遮挡、调试图输出及新旧任务配置的等价性。历史图二维演示结果不能代替实际抓取验证。

## 2026-09-22 局部断边修正

面板匹配通过、局部底板报 `No complete plate contour` 时，可能是细边框被全局阈值分割成不闭合的线。新版本在局部校正图上组合 Otsu 与自适应阈值，并仅以 5×5 / 3×3 核修补短断口；候选仍须通过原有尺寸/位置检查和四边未修补证据检查。长边缺失、目标移除或轴帽遮挡仍拒绝。

调试输出新增 `*_panel_layout_color.png` 原始图和同名 JSON 元数据；元数据包含精确投影矩阵、候选计数、几何/边缘拒绝计数及选用的分割方法。排查时优先使用原图，叠加图的标记会影响二值化阈值。


## Fresh-frame sampling and multiple references (2026-09-23)

The default `ros2_qr_panel_layout_movel_reroute.yaml` now enables `vision.fresh_sampling` and five `panel_layout_templates`. The original template and all legacy modes remain available. Four additional references (largest/smallest/left/right apparent views) come from visually reviewed historical images; `config/panel_layout/template_sources.json` records provenance. Every template retains the original inlier, spatial span, reprojection, plate and hub checks. Scene SIFT features are extracted once per frame. If accepted templates disagree by more than 5 pixels on the shaft center, the frame is rejected.

Sampling starts after motion/settling and uses detached color/depth references. Both header timestamps must advance beyond the entry boundary and previous consumed pair; frame age must be <=0.75 seconds, inter-stream skew <=0.15 seconds, and timestamps must not be more than 0.1 seconds in the future. This expects ROS-compatible timestamps; the installed Orbbec driver defaults to device time synchronized to the host every 30 seconds. Timestamp faults are reported, not silently bypassed.

The default acquisition budget is 6 seconds and at most 5 attempts. Each fresh-pair wait is capped at 2 seconds. A running OpenCV operation or diagnostic file write is not forcibly interrupted, so wall duration can exceed the acquisition budget by the final operation. Two consecutive accepted frames must agree within 5 pixels and 15 mm depth. Rejected frames reset confirmation. TF, configuration, depth-range and workspace faults are not retried. Neither retries nor confirmation issue any robot commands. The resolver owns frame waiting for this mode, including alignment and intermediate reacquisition; original movement ordering is unchanged.

Failed attempts save raw color, available plate/axis overlay, match visualization and JSON evidence under `vision_debug/`, even when `save_debug_images: false`. Frame timeout metadata explicitly marks cached images as stale/invalid. Metadata contains color/depth timestamps, ages, skew, rejection reason, and per-template quality; match evidence contains individual inlier flags. Only confirmed targets are published. Legacy mode behavior is selected by using its original YAML or disabling this sampling policy.

Validation: 285 held-out historical frames (excluding four new reference source images): original single-template code accepted 266; the bank accepted those same 266 plus 11 previously rejected frames; 8 remained rejected. Maximum axis change on jointly accepted frames was 0.163 pixels. All 11 recovered images were visually reviewed. This is offline 2D localization evidence, not an end-to-end robot success-rate claim. Recent 17:33 failure images were not saved and could not be replayed.


### Reference from a previously rejected distant view (2026-09-23)

The default bank now contains six templates. `template_distant_panel.json` is manually annotated from `middle_black_knob_20260922_175642_078985_panel_layout_color.png`, with the same validation thresholds. The source frame now passes 2D localization. On the other 288 historical frames, the previously accepted 281 remain accepted and seven remain rejected. This is a no-regression check; it does not establish improved detection on unseen images of this distant view.

Two other failure images (20260923 15:38:30 and 15:49:34) were manually annotated as candidates but are NOT enabled: self-localization still fails plate validation / unique hub identification. Candidates and diagnostics are retained in the `.cabinet_backups/failed_templates_20260923_194006/candidates` directory. The 15:30:03 image has no cabinet in view and must remain a negative example; it is now covered by a regression test. Movement logic and detection thresholds were not changed.

## 独立双旋钮模式（2026-09-23）

`ros2_qr_panel_layout_two_knobs.yaml` 是新增模式，原有
`ros2_qr_panel_layout_movel_reroute.yaml` 与默认启动入口保持不变。

```bash
ros2 launch realman_gemini_cabinet direct_driver_two_knobs.launch.py
```

任务仍由 `/rg_cabinet/run_sequence_from_qr` 触发。执行顺序为：读表 →
中间方底座旋钮按原设置旋转 90°、释放并撤退 → 上排黄色按钮右侧黑色旋钮旋转
+60°（工具 z 轴、方向沿用原设置）、释放并撤退，最后执行配置中的安全位。
右侧在原快速接近之后、慢速接触之前，新增接触前居中；中间旋钮保持原流程。

新增目标 ID 为 `upper_row_right_black_knob`，不是下排右端旋钮。
它使用独立的圆形底座检测：全局模板预测区域后，在局部验证暗色圆边缘、覆盖率、
缺口、内外明暗差与尺寸；圆形中心作为转轴估计，不把把手重心或反光点作为轴心。
黄色框为预测区域，绿色轮廓为实测圆形底座，品红十字为轴心估计。
多模板结果冲突、遮挡或目标太小时拒绝定位；无颜色块回退。
圆形结果 `plate_corners` 为 null，`knob_outline_px` 保存圆周，
`local_bbox_corners` 仅用于沿用现有深度采样。

右侧旋钮的观察位姿来自操作者提供的数据，按当前驱动 `base_link` 存储：
位置 `[-0.3334310054779053, -0.0026940000243484974, 0.5271419882774353]`；
四元数 xyzw `[0.0006479999865405262, -0.7373149991035461,
-0.011230000294744968, 0.6754540205001831]`。
`observation_pose.source: fixed` 表示保留配置中的位姿、不消费或覆盖二维码位置；
它仍然执行观察位运动。原有两个观察位继续从二维码加载。

模板采用机器上无标记的历史原始图：`reference_left.png`、
`reference_right.png`、`reference_smallest.png`。操作者提供的带彩色框截图用于确认
目标布局，未把截图标记当成匹配特征。历史图和离线测试仅验证二维定位与软件流程；
真实夹取、深度精度以及新观察位间的机械臂运动仍需现场验证。


## 右侧旋钮接触前居中（2026-09-24）

仅双旋钮配置中的 `upper_row_right_black_knob` 启用 `precontact_centering`。
流程：原观察位/远距离对齐 → 原快速接近 → 两帧新图定位 → 小幅 Y/Z 修正
→ 再次两帧复查 → 慢速接触 → 夹紧 → 沿原方向转 60° → 释放/撤退。
原单旋钮配置和中间旋钮不开启此功能。

- 每轴视觉估计误差 ≤ 2 mm 才进入慢速接触；这不是物理精度保证。
- 修正最多两次，观测偏差每轴不超过 30 mm，每次修正每轴不超过 15 mm，累计 Y/Z 路程不超过 40 mm，速度 5 mm/s。
- 近距离修正只做 Y/Z 直线 MoveL，逐段做原关节预算；禁止观察位的 10 cm 自动绕行。
- 保留原模板质量、深度、工作区和两帧一致性门限；识别失败、不收敛或预算失败时不慢速进给、不夹紧。
- 成功复查的原图和叠加图保存在 `vision_debug`；任务返回的 `precontact_centering` 记录每次误差、修正量与复查结果。
- 未修改 TCP/手眼标定、深度取样方式、原 X 进给距离或慢速接触距离。
- 近距离仍需实机验证目标是否可见以及模板是否可匹配；离线流程测试不证明实物夹持已居中。


### 近距离视野适配（2026-09-24）

右侧 `params.precontact_panel_layout_templates` 仅在接触前居中阶段覆盖模板列表，
指向 `config/panel_layout_right_knob/precontact/template.json`。
近距离上排仪表已出视野，改以当前可见开关/按钮注册面板，再局部拟合上排黑色圆盘。
远距离原八模板、匹配阈值、深度/工作区门限、两帧确认和修正范围均保持原值。


### 近距离修正范围（2026-09-24 13:24 实测后）

近距离两帧确认后测得 Y=-10.051 mm、Z=+18.175 mm，触发原 15 mm 单轴上限；
按先 Y 后 Z 的执行路径，所需累计路程为 28.226 mm，也超过原 20 mm 累计上限。
仅当前双旋钮模式右侧改为单轴 20 mm、累计 35 mm；类默认值保持原值。
仍最多两次修正、速度 5 mm/s、每轴残差 ≤2 mm 才能进入慢速接触；
仍执行 MoveL 关节预算并禁止近处自动绕行。实际夹持居中需现场复测。


### 分步居中修正（2026-09-24 13:28/13:32 实测后）

原实现把单次指令距离上限也当作观测偏差上限，20.619/22.921 mm 的 Z 误差
在运动前被直接拒绝。当前右侧模式将二者分开：max_error_m=30 mm，
max_axis_m=15 mm，max_total_m=40 mm，仍最多两次修正。
每轮先确认两帧、检查误差/剩余预算、将每轴指令截在 15 mm 内、做 MoveL 关节预算，
然后执行并重新采样实际残差；不按旧误差盲目执行剩余量。
例如 Y=-13.995、Z=22.921 mm，第一轮最多 Y=-13.995、Z=15 mm；
第二轮方向和距离完全由新图决定；最终仍须每轴≤2 mm 才接触。
超过观测范围、累计预算不足、识别或关节预算失败、两轮后未居中均停止。
该功能不能替代手眼/TCP 标定，也未增加环境碰撞检测。
