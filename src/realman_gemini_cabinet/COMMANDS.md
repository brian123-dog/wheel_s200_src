# realman_gemini_cabinet 命令速查

本文件集中保存编译、启动、调用服务和排查用的命令,方便直接复制。
工作区根目录约定为 `/home/sunrise/wheel_S200_src`,ROS 发行版为 `humble`。

> 复制提示:多行命令中的 `\` 续行可原样粘贴。带 `#` 的备注行不会被执行。

---

## 0. 一次性环境准备

```bash
source /opt/ros/humble/setup.bash
export WS=/home/sunrise/wheel_S200_src
cd $WS
```

后续所有命令都假设已经执行过上面的 `source` 和 `cd`。

---

## 1. 编译

### 1.1 只编译本功能包(最小改动)

```bash
cd /home/sunrise/wheel_S200_src
colcon build --base-paths src --packages-up-to realman_gemini_cabinet
source install/setup.bash
```

### 1.2 按需带上依赖包

```bash
# 只跑 mock 上层任务
colcon build --base-paths src --packages-up-to realman_gemini_cabinet

# 要接 Orbbec 相机
colcon build --base-paths src --packages-up-to orbbec_camera realman_gemini_cabinet

# 要接 RealMan bringup / driver
colcon build --base-paths src --packages-up-to rm_bringup rm_driver realman_gemini_cabinet

# 要接 MoveIt2(RM65 示例)
colcon build --base-paths src --packages-up-to rm_65_config realman_gemini_cabinet
```

### 1.3 全量编译

```bash
cd /home/sunrise/wheel_S200_src
colcon build --base-paths src
source install/setup.bash
```

### 1.4 修改配置后需要重新编译

`config/*.yaml`、`launch/*.launch.py`、`README.md` 等通过 `setup.py` 的 `data_files`
安装到 `install/.../share/`。**改了这些文件必须重新 `colcon build` 并重新 `source`**,
否则 launch 读取的仍是旧副本。

只想改配置、不想编译时,可以改用源码路径直接传入:

```bash
task_config:=/home/sunrise/wheel_S200_src/src/realman_gemini_cabinet/config/ros2_qr_panel_layout_movel_reroute.yaml
```

---

## 2. 启动

### 2.1 查询安装路径(写 `task_config` 时常用)

```bash
ros2 pkg prefix realman_gemini_cabinet
# 拼接出配置绝对路径
echo "$(ros2 pkg prefix realman_gemini_cabinet)/share/realman_gemini_cabinet/config"
```

### 2.2 Mock 演示(不接硬件,最安全)

```bash
ros2 launch realman_gemini_cabinet mock_demo.launch.py
```

带相机、带机械臂 bringup:

```bash
ros2 launch realman_gemini_cabinet mock_demo.launch.py include_camera:=true

ros2 launch realman_gemini_cabinet mock_demo.launch.py \
  include_rm_bringup:=true \
  rm_bringup_launch:=robot_with_gripper_bringup.launch.py
```

### 2.3 硬件启动(hardware_bringup)

默认 `backend:=rm_driver`,默认会拉起 `rm_bringup`,并且默认夹爪节点开启。

```bash
ros2 launch realman_gemini_cabinet hardware_bringup.launch.py
```

### 2.4 直接驱动启动(direct_driver_bringup,现场当前默认)

该 launch **默认加载 `ros2_qr_panel_layout_movel_reroute.yaml`**(已启用对齐自动绕行),
`rm_driver`、robot_description 默认开启。

```bash
ros2 launch realman_gemini_cabinet direct_driver_bringup.launch.py
```

### 2.5 指定任务配置启动

```bash
ros2 launch realman_gemini_cabinet direct_driver_bringup.launch.py \
  task_config:=$(ros2 pkg prefix realman_gemini_cabinet)/share/realman_gemini_cabinet/config/ros2_qr_panel_layout_movel_check.yaml
```

用源码目录里的配置(免编译):

```bash
ros2 launch realman_gemini_cabinet hardware_bringup.launch.py \
  task_config:=/home/sunrise/wheel_S200_src/src/realman_gemini_cabinet/config/ros2_vision_button_demo.yaml
```

### 2.6 常用 launch 参数

| 参数 | mock_demo 默认 | hardware_bringup 默认 | direct_driver_bringup 默认 |
| --- | --- | --- | --- |
| `task_config` | `config/ros2_mock_demo.yaml` | `config/ros2_qr_named_panel_demo.yaml` | `config/ros2_qr_panel_layout_movel_reroute.yaml` |
| `backend` | `mock` | `rm_driver` | `rm_driver` |
| `include_camera` | `false` | `false` | — |
| `include_rm_bringup` | `false` | `true` | — |
| `include_rm_driver` | — | — | `true` |
| `include_robot_description` | — | — | `true` |
| `include_static_camera_tf` | — | — | `false` |
| `publish_gripper_joint_states` | `false` | `true` | `false` |
| `rm_bringup_launch` | `robot_with_gripper_bringup.launch.py` | 同左 | — |
| `rm_driver_launch` | — | — | `rm_63_driver.launch.py` |
| `robot_description_launch` | — | — | `robot_with_gripper.launch.py` |
| `camera_color_topic` | `/camera/color/image_raw` | `/camera_gemini/color/image_raw` | `/camera_gemini/color/image_raw` |
| `camera_depth_topic` | `/camera/depth/image_raw` | `/camera_gemini/depth/image_raw` | `/camera_gemini/depth/image_raw` |
| `camera_info_topic` | `/camera/color/camera_info` | `/camera_gemini/color/camera_info` | `/camera_gemini/color/camera_info` |
| `camera_frame` | `camera_color_optical_frame` | `camera_gemini_color_optical_frame` | `camera_gemini_color_optical_frame` |
| `vision_enabled` | `true` | `true` | `true` |
| `vision_tf_timeout_s` | `1.0` | `1.0` | `1.0` |
| `qr_result_file` | 空 | 空 | 空 |
| `qr_results_dir` | 空(节点内自动推导) | 空 | 空 |
| `base_frame` | — | — | `base_link` |

`direct_driver_bringup.launch.py` 还包含夹爪相关参数(默认值):

`gripper_backend:=socket_json`、`gripper_ip:=192.168.1.18`、`gripper_port:=8080`、
`gripper_device_id:=1`、`gripper_pos_open:=0`、`gripper_pos_close:=9000`、
`gripper_command_timeout_s:=2.0`、`gripper_modbus_port:=1`、`gripper_baudrate:=115200`、
`gripper_voltage_type:=3`、`gripper_default_speed:=100`、`gripper_default_force:=50`、
`gripper_acceleration:=1000`、`gripper_deacceleration:=1000`、`gripper_init_on_connect:=true`。

---

## 3. 任务配置一览(config/)

| 配置 | 说明 |
| --- | --- |
| `cabinet_demo.yaml` | 纯 Python CLI 用的示例序列 |
| `ros2_mock_demo.yaml` | mock 节点默认配置 |
| `ros2_qr_cabinet_demo.yaml` | QR 基础柜体演示 |
| `ros2_qr_named_panel_demo.yaml` | 命名面板 + 黑色色块定位(旧路径,保留) |
| `ros2_qr_panel_layout_demo.yaml` | 启用 `locator: panel_layout`(SIFT 模板定位) |
| `ros2_qr_panel_layout_movel_check.yaml` | 在 panel_layout 基础上加 `alignment_preflight: true`(仅预检查) |
| `ros2_qr_panel_layout_movel_reroute.yaml` | 预检查 + `alignment_reroute: true`(对齐自动绕行,**现场当前使用**) |
| `ros2_qr_vision_cabinet_demo.yaml` | 相机目标相机源 |
| `ros2_vision_button_demo.yaml` | 视觉按钮演示(调色域/ROI 用) |

切换方式(只需替换 `task_config`):

```bash
# 切到自动绕行(默认)
task_config:=$(ros2 pkg prefix realman_gemini_cabinet)/share/realman_gemini_cabinet/config/ros2_qr_panel_layout_movel_reroute.yaml

# 切回仅预检查
task_config:=$(ros2 pkg prefix realman_gemini_cabinet)/share/realman_gemini_cabinet/config/ros2_qr_panel_layout_movel_check.yaml

# 切回旧面板定位
task_config:=$(ros2 pkg prefix realman_gemini_cabinet)/share/realman_gemini_cabinet/config/ros2_qr_named_panel_demo.yaml
```

面板模板资源:`config/panel_layout/template.json` 与 `config/panel_layout/reference.png`,
模板内 `reference_image` 相对模板 JSON 所在目录解析,源码目录和安装目录都可用。

---

## 4. 服务调用(节点运行中)

### 4.1 触发任务

```bash
# 按当前 task_config 完整执行
ros2 service call /rg_cabinet/run_sequence std_srvs/srv/Trigger "{}"

# 从 QR 结果文件解析目标后执行
ros2 service call /rg_cabinet/run_sequence_from_qr std_srvs/srv/Trigger "{}"

# 只做视觉解算,不动机器人(调试视觉时先跑这个)
ros2 service call /rg_cabinet/resolve_targets std_srvs/srv/Trigger "{}"
```

### 4.2 夹爪

```bash
ros2 service call /rg_cabinet/open_gripper std_srvs/srv/Trigger "{}"
ros2 service call /rg_cabinet/close_gripper std_srvs/srv/Trigger "{}"
ros2 service call /rg_cabinet/set_gripper std_srvs/srv/SetBool "{data: true}"
```

话题方式控制夹爪开合度(0.0~1.0):

```bash
ros2 topic pub --once /rg_cabinet/gripper_position_cmd std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic pub --once /rg_cabinet/gripper_position_cmd std_msgs/msg/Float64 "{data: 0.0}"
```

### 4.3 全部服务一览

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/rg_cabinet/run_sequence` | `std_srvs/srv/Trigger` | 执行当前任务序列 |
| `/rg_cabinet/run_sequence_from_qr` | `std_srvs/srv/Trigger` | 从 QR 结果驱动任务 |
| `/rg_cabinet/resolve_targets` | `std_srvs/srv/Trigger` | 仅视觉解算,不运动 |
| `/rg_cabinet/set_gripper` | `std_srvs/srv/SetBool` | 设置夹爪状态 |
| `/rg_cabinet/open_gripper` | `std_srvs/srv/Trigger` | 张开夹爪 |
| `/rg_cabinet/close_gripper` | `std_srvs/srv/Trigger` | 夹紧夹爪 |

---

## 5. 话题查看与调试

### 5.1 查看任务状态

```bash
ros2 topic echo /rg_cabinet/status
```

`/rg_cabinet/status` 为 `std_msgs/msg/String`,包含人类可读摘要与 `event` 字段
(`node_ready`、`task_stage`、`alignment_preflight`、失败原因等)。

### 5.2 查看视觉解算出的目标位姿

```bash
ros2 topic echo /rg_cabinet/target_pose
```

`/rg_cabinet/target_pose` 为 `geometry_msgs/msg/PoseStamped`,发布视觉解算并变换到
`base_frame` 后的目标位姿,配合 `/rg_cabinet/resolve_targets` 可在不动机器人的前提下校验。

### 5.3 查看机械臂位姿与力/电流

```bash
ros2 topic echo --once /rm_driver/udp_arm_position
ros2 topic echo --once /rm_driver/udp_six_force
ros2 topic echo --once /rm_driver/udp_joint_current
```

### 5.4 相机话题自检

```bash
ros2 topic hz /camera_gemini/color/image_raw
ros2 topic hz /camera_gemini/depth/image_raw
ros2 topic echo --once /camera_gemini/color/camera_info
```

### 5.5 TF 检查

```bash
ros2 run tf2_ros tf2_echo base_link camera_gemini_color_optical_frame
ros2 run tf2_tools view_frames
```

### 5.6 手动发运动指令(谨慎,会让机械臂动)

```bash
ros2 topic pub --once /rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
```

`rm_driver` 后端使用的话题:

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/rm_driver/movel_cmd` | `rm_ros_interfaces/msg/Movel` | 位姿直线运动(含短伺服运动) |
| `/rm_driver/movej_p_cmd` | `rm_ros_interfaces/msg/Movejp` | 关节角目标(位姿式) |
| `/rm_driver/movej_cmd` | `rm_ros_interfaces/msg/Movej` | 关节运动 |
| `/rm_driver/set_gripper_position_cmd` | `rm_ros_interfaces/msg/Gripperset` | 夹爪位置 |
| `/rm_driver/move_stop_cmd` | `std_msgs/msg/Empty` | 停止 |

> 三代控制器不支持 `rm_movel_offset`,短伺服默认走 `movel_cmd`。
> 仅当控制器支持 `/rm_driver/movel_offset_cmd` 时才设
> `rm_driver_servo_motion_mode:=movel_offset`。

---

## 6. 纯 Python CLI(不启动 ROS 节点)

```bash
cd /home/sunrise/wheel_S200_src/src/realman_gemini_cabinet
PYTHONPATH=src python3 -m rg_cabinet.task_runner --config config/cabinet_demo.yaml --mock
```

编译后可用的控制台脚本:

```bash
ros2 run realman_gemini_cabinet rg_cabinet_demo --config config/cabinet_demo.yaml --mock
ros2 run realman_gemini_cabinet cabinet_task_node
ros2 run realman_gemini_cabinet gripper_joint_state_node
```

> CLI 目前只接通 `--mock`;不传 `--mock` 会直接报错退出,避免误动真机。

---

## 7. 测试

### 7.1 全量单元测试

```bash
cd /home/sunrise/wheel_S200_src/src/realman_gemini_cabinet
ROS_LOG_DIR=/tmp/rg_cabinet_ros_logs PYTHONPATH=src:$PYTHONPATH \
  python3 -m unittest discover -s tests
```

### 7.2 pytest(需禁用第三方插件自动加载)

设备的用户级 anyio pytest 插件与系统 pytest 不兼容,必须加
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。

```bash
cd /home/sunrise/wheel_S200_src/src/realman_gemini_cabinet
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:$PYTHONPATH \
  python3 -m pytest -q tests
```

### 7.3 单项测试

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:$PYTHONPATH python3 -m pytest -q \
  tests/test_panel_layout_mode.py \
  tests/test_vision_target_resolver.py \
  tests/test_image_tools.py \
  tests/test_launch_defaults.py
```

### 7.4 colcon 测试

```bash
cd /home/sunrise/wheel_S200_src
colcon test --packages-select realman_gemini_cabinet
colcon test-result --verbose
```

常用测试文件对照:

| 文件 | 覆盖内容 |
| --- | --- |
| `test_launch_defaults.py` | launch 默认参数 |
| `test_vision_target_resolver.py` | 视觉解算、深度/工作区拒绝 |
| `test_panel_layout_mode.py` | `locator: panel_layout` 模式 |
| `test_movel_alignment_check.py` | 对齐预检查 |
| `test_movel_alignment_routes.py` | 对齐自动绕行 |
| `test_rm_driver_backend.py` | rm_driver 话题与消息 |
| `test_gripper_socket.py` | socket JSON 夹爪 |
| `test_task_reporting.py` | 状态摘要与诊断输出 |

---

## 8. 进程管理与清理

```bash
# 查看相关进程
pgrep -af "ros2 launch realman_gemini_cabinet|rm_driver|rviz2|move_group|component_container|cabinet_task_node"

# 看节点清单
ros2 node list
ros2 node info /cabinet_task_node

# 优雅停止
pkill -f "ros2 launch realman_gemini_cabinet"
pkill -f cabinet_task_node
pkill -f rm_driver

# 查看 launch 参数
ros2 launch realman_gemini_cabinet direct_driver_bringup.launch.py --show-args
```

---

## 9. tmux 多窗格常用组合

一次启动多个终端时建议用 tmux,断线不丢任务。

```bash
# 新建会话
tmux new -s cabinet

# 在 tmux 内:左右分屏 / 上下分屏
#   Ctrl+b %      Ctrl+b "
#   Ctrl+b 方向键  切换窗格
#   Ctrl+b z       当前窗格最大化/还原
#   Ctrl+b d       分离(任务继续跑)
#   tmux a -t cabinet   重新接入
```

典型三窗格布局:

```bash
# 窗格1:启动驱动 + 任务节点
ros2 launch realman_gemini_cabinet direct_driver_bringup.launch.py

# 窗格2:盯状态
ros2 topic echo /rg_cabinet/status

# 窗格3:发服务 / 看位姿
ros2 service call /rg_cabinet/run_sequence std_srvs/srv/Trigger "{}"
ros2 topic echo --once /rm_driver/udp_arm_position
```

---

## 10. 常见问题排查

### 10.1 配置改了但行为没变

原因:节点读的是 `install/.../share/.../config` 里的旧副本。
处理:重新 `colcon build --packages-up-to realman_gemini_cabinet` 并重新 `source install/setup.bash` 后再启动;
或直接把 `task_config` 指向源码目录里的 yaml。

### 10.2 `PREFLIGHT_UNAVAILABLE`

SDK 缺失或没有新鲜静止的 `/joint_states`、`tcp_pose_topic` 反馈。
这是**拒绝放行**,不会绕过检查。先确认 `rm_driver` 已启动且反馈正常。

默认 SDK 路径由 `rm_driver` 安装前缀推导为工作空间
`src/ros2_rm_robot/rm_driver/lib/libapi_cpp.so`;
其他目录需在技能参数中指定 `alignment_preflight_sdk_library` 绝对路径。

### 10.3 `MARGIN_TOO_SMALL` / `JOINT_BRANCH_JUMP`

- `MARGIN_TOO_SMALL`:预测路径上关节限位余量不足。
- `JOINT_BRANCH_JUMP`:相邻采样点出现大幅关节跳变。
- `START_OUTSIDE_MODEL_LIMITS`:实际角度已超出出厂算法模型范围,需核对现场限位。
- `MODEL_OR_TCP_FRAME_MISMATCH`:正运动学与回读位姿不一致。

这些阈值是可配置的保守筛选值,不是厂家保证的安全距离,不能靠放大阈值掩盖型号/标定问题。

### 10.4 自动绕行失败

`alignment_reroute` 的搜索是有边界的候选搜索,不保证任意停放姿态都有路线。
`alignment_reroute_drop_clearance_m` 是现场自由空间约束(已确认观察位下方约 10 cm 有空间),
现场空间变了要相应更新;程序不会扩大控制器限位或盲目重试。

### 10.5 视觉定位失败

启用 `save_debug_images: true` 时,调试图输出在 `vision_debug_dir`(默认 `vision_debug`):
`*_panel_layout.png` 中黄色为预测区域、绿色为底板、紫色为轴帽中心。

```bash
ls -lt /home/sunrise/wheel_S200_src/vision_debug | head
```

常见拒绝原因:

- `No complete plate contour`:细边框被全局阈值切成不闭合的线(新版本已用 Otsu + 自适应阈值 + 5×5/3×3 核修补短断口)。
- `Missing or ambiguous central hub cap`:旋钮轴心缺失或存在多个候选。
- `outside workspace`:目标坐标超出 `vision.workspace_bounds`。

### 10.6 相机不通

示例配置里相机命名可能是 `camera` 或 `camera_gemini`,先确认实际话题名:

```bash
ros2 topic list | grep -i camera
ros2 topic hz $(ros2 topic list | grep -m1 'color/image_raw')
```

---

## 11. 备份与打包

```bash
cd /home/sunrise/wheel_S200_src/src
zip -r realman_gemini_cabinet.zip realman_gemini_cabinet/

# 排除缓存
zip -r realman_gemini_cabinet.zip realman_gemini_cabinet/ \
  -x '*/__pycache__/*' '*.pyc' '*/.pytest_cache/*'
```

工作区中已有自动备份目录:`.cabinet_backups/<标签>_<时间戳>/`。

---

## 12. 相关文档

| 文件 | 内容 |
| --- | --- |
| `README.md` | 包定位、ROS2 接口、硬件集成点 |
| `PANEL_LAYOUT_MODE.md` | `locator: panel_layout` 模板定位模式 |
| `ALIGNMENT_PREFLIGHT.md` | MoveL 对齐预检查 |
| `ALIGNMENT_REROUTE.md` | 对齐阶段自动搜索多段 MoveL |
| `cli.md` | 现场零散命令记录(历史) |
| `COMMANDS.md` | 本文件 |
