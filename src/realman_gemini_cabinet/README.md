# RealMan + Gemini2 Cabinet Manipulation

This package turns the repository-selection plan into a small, testable
engineering scaffold for electrical-cabinet manipulation.

It intentionally does not vendor or modify `robothon2023`,
`DesignLearnRG_euROBIN`, `robotlearningblock`, `ManiSkill`, or `where2act`.
Instead, it reuses their useful ideas:

- `DesignLearnRG_euROBIN`: task-level sequencing with parameterized motions.
- `robothon2023`: guarded contact motions for buttons, sliders, knobs, and doors.
- `robotlearningblock`: explicit state verification as a first-class concept.
- `ManiSkill`: future simulation/data path, not needed for the first real-robot loop.
- `where2act`: research reference for discovering push/pull affordances.

## Package Layout

- `rg_cabinet.adapters`: robot and RGB-D camera interfaces, plus mock and client
  wrappers for RealMan/Gemini2 SDK integration.
- `rg_cabinet.perception`: simple RGB-D color-blob detector and depth
  back-projection into a shared target representation.
- `rg_cabinet.skills`: button press, toggle/slider, knob turn, and door/handle
  task primitives with guarded low-speed contact.
- `rg_cabinet.verification`: task-board or cabinet-state verification hooks.
- `rg_cabinet.task_runner`: YAML-driven sequence executor.

## Run the Mock Demo

```bash
cd /home/jxs/JxsWorkspace/manipulator_ws/src/realman_gemini_cabinet
PYTHONPATH=src python3 -m rg_cabinet.task_runner --config config/cabinet_demo.yaml --mock
```

## ROS2 Mock And Hardware Demos

The repository is also a ROS2 `ament_python` package. The mock path loads the
same YAML sequence and publishes status. The hardware path uses `rm_driver` and
therefore sends real RealMan motion only after `/rg_cabinet/run_sequence` is
called.

```bash
cd /home/jxs/JxsWorkspace/manipulator_ws
colcon build --base-paths src --packages-up-to realman_gemini_cabinet
source install/setup.bash
ros2 launch realman_gemini_cabinet mock_demo.launch.py
```

`--base-paths src` matches the standard ROS2 workspace layout and keeps `colcon`
focused on the ROS packages under `src/`. `--packages-up-to` also builds the
RealMan message package used for force/current status.

This mock launch does not start the RealMan arm by default. To include the arm
bringup explicitly:

```bash
ros2 launch realman_gemini_cabinet mock_demo.launch.py include_rm_bringup:=true rm_bringup_launch:=robot_with_gripper_bringup.launch.py
```

The ROS2 node is `cabinet_task_node`. It provides:

- service: `/rg_cabinet/run_sequence` (`std_srvs/srv/Trigger`)
- service: `/rg_cabinet/resolve_targets` (`std_srvs/srv/Trigger`, vision only, no motion)
- status topic: `/rg_cabinet/status` (`std_msgs/msg/String`)
- default backend: `mock`
- default task config: `config/ros2_mock_demo.yaml`

## Vision-Guided Target Poses

Task steps can request a live Gemini2 target by setting `target.source: camera`.
The node then detects a configured color blob, reads median depth around the
detected center, back-projects the pixel with `/camera/color/camera_info`,
transforms the point into `base_link` with tf2, publishes the resolved pose on
`/rg_cabinet/target_pose`, and passes that pose into the normal skill executor.

Example:

```bash
ros2 launch realman_gemini_cabinet hardware_bringup.launch.py \
  task_config:=/home/jxs/JxsWorkspace/manipulator_ws/install/realman_gemini_cabinet/share/realman_gemini_cabinet/config/ros2_vision_button_demo.yaml
```

Tune `rgb_min`, `rgb_max`, `roi`, `depth_scale_m`, `offset_base_m`, and
`rpy_base` in `config/ros2_vision_button_demo.yaml` before enabling real motion.
Use `/rg_cabinet/resolve_targets` first to validate the detected pose without
moving the arm.
For vertical cabinet panels, set skill params such as
`approach_offset_frame: tool` so pre-contact poses are generated along the
target/tool approach axis instead of the global `base_link` z axis.

Camera topics are prepared for Orbbec Gemini2 defaults:

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`

Optional launch arguments:

```bash
ros2 launch realman_gemini_cabinet mock_demo.launch.py include_camera:=true
ros2 launch realman_gemini_cabinet mock_demo.launch.py include_rm_bringup:=true rm_bringup_launch:=robot_with_gripper_bringup.launch.py
```

`backend:=rm_driver` publishes pose motion to `/rm_driver/movel_cmd`, joint-pose
motion to `/rm_driver/movej_p_cmd`, joint motion to `/rm_driver/movej_cmd`, and
gripper commands to `/rm_driver/set_gripper_position_cmd`. Short servo motions
also use `/rm_driver/movel_cmd` by default because third-generation controllers
do not support `rm_movel_offset`; set `rm_driver_servo_motion_mode:=movel_offset`
only on controllers that support `/rm_driver/movel_offset_cmd`.
`backend:=moveit2` is currently a safe placeholder. It keeps RealMan MoveIt2
parameters available (`rm_group`, `/rm_group_controller/follow_joint_trajectory`,
`base_link`, `Link6`) and refuses motion until the real MoveIt2/action wiring is
implemented.

## Run Tests

```bash
cd /home/jxs/JxsWorkspace/manipulator_ws/src/realman_gemini_cabinet
ROS_LOG_DIR=/tmp/rg_cabinet_ros_logs PYTHONPATH=src:$PYTHONPATH python3 -m unittest discover -s tests
```

## Hardware Integration Points

Use `RealManAdapter` by passing a RealMan SDK client object and, if needed, a
`RealManCommandMap` that maps the generic methods to your SDK method names:

```python
from rg_cabinet.adapters.realman import RealManAdapter, RealManCommandMap

robot = RealManAdapter(
    client=realman_client,
    commands=RealManCommandMap(
        move_pose="move_pose",
        move_joint="move_joint",
        servo_twist="servo_twist",
        stop="stop",
        set_gripper="set_gripper",
        get_tcp_pose="get_tcp_pose",
        get_wrench="get_wrench",
        get_joint_currents="get_joint_currents",
    ),
)
```

Use `Gemini2Camera` the same way with a Gemini2 SDK client. The current wrapper
accepts either a single `get_rgbd_frame()` style method returning a dict, or
separate color/depth/intrinsics methods configured in `Gemini2CommandMap`.

The first real-robot milestone should keep `--mock` passing, then replace only
the adapters while leaving `skills` and `task_runner` untouched.
