// arm_motion_executor.hpp
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int32.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ArmMotionExecutor
{
public:
  explicit ArmMotionExecutor(rclcpp::Node * node);

  bool initialize();

  void add_scene_objects();
  void delete_cup_obstacle(int cup_id);
  void add_cup_obstacle(int cup_id);

  bool control_gripper_open();
  bool control_gripper_close();

  bool move_to_pose(const geometry_msgs::msg::Pose & pose);
  bool move_linear(
    const geometry_msgs::msg::Pose & target_pose,
    double vel_scale = 1.0,
    double acc_scale = 1.0);

  bool move_arc(
    const geometry_msgs::msg::Pose & mid_pose,
    const geometry_msgs::msg::Pose & end_pose,
    int steps = 50,
    bool continuous = true,
    double vel_scale = 1.0,
    double acc_scale = 1.0);

  bool rotate_end_effector();

  /// 按指定角度旋转末端执行器(绕 joint6 Z 轴)
  /// @param degrees 旋转角度(度)，正值为正向旋转
  /// @return 成功返回 true
  bool rotate_end_effector_by_angle(double degrees);

  bool get_cup_pose(
    int cup_id,
    geometry_msgs::msg::Pose & pre_grasp,
    geometry_msgs::msg::Pose & grasp,
    geometry_msgs::msg::Pose & up_grasp) const;

private:
  rclcpp::Node * node_;

  // MoveIt clients
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
  rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr execute_client_;
  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr cartesian_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_client_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;

  // cup scene topics
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cup_delete_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cup_add_pub_;

  bool send_goal_and_wait(const moveit_msgs::action::MoveGroup::Goal & goal_msg);
  bool execute_trajectory(const moveit_msgs::msg::RobotTrajectory & traj);
  bool get_current_pose(geometry_msgs::msg::Pose & pose_out) const;
  bool init_gripper();
  bool send_gripper_json(const std::string & payload) const;
  static geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  int gripper_socket_fd_ {-1};
};