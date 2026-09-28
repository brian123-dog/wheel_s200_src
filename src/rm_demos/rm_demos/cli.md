cd ros2_ws

source install/setup.bash 

ros2 launch rm_bringup robot_with_gripper_bringup.launch.py

ros2 run rm_demos realsense_publisher 

ros2 run rm_demos fivecups_spawn  

ros2 run rm_demos five_demos



# ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: true}"

# ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: false}"
