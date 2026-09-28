// align_to_wall_action_bt_node.hpp
// BT ActionNode that calls /align_to_wall action and blocks until completion.

#pragma once

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces/action/align_to_wall.hpp"

namespace multi_action_node
{

class CallAlignToWall : public BT::StatefulActionNode
{
public:
  using AlignToWall = base_interfaces::action::AlignToWall;
  using GoalHandleAlign = rclcpp_action::ClientGoalHandle<AlignToWall>;

  CallAlignToWall(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("target_distance", 0.30, "Target wall distance (m)"),
      BT::InputPort<double>("target_yaw", 0.0, "Target yaw angle (rad)"),
      BT::InputPort<double>("tolerance_distance", 0.02, "Distance tolerance (m)"),
      BT::InputPort<double>("tolerance_yaw", 0.017, "Yaw tolerance (rad)"),
      BT::InputPort<std::string>("laser_frame", "laser_link", "Laser TF frame"),
      BT::InputPort<std::string>("camera_frame", "camera_link", "Camera TF frame"),
      BT::InputPort<int>("server_timeout", 30000, "Timeout in ms"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<AlignToWall>::SharedPtr action_client_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;

  bool goal_completed_{false};
  bool result_success_{false};
  std::string result_message_;
  std::chrono::milliseconds server_timeout_{30000};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace multi_action_node
