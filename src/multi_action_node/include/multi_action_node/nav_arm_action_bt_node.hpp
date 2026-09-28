// nav_arm_action_bt_node.hpp
// BT ActionNode that calls /nav_arm_control action and blocks.

#pragma once

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces/action/nav_arm_control.hpp"

namespace multi_action_node
{

class CallNavArmAction : public BT::StatefulActionNode
{
public:
  using NavArmControl = base_interfaces::action::NavArmControl;
  using GoalHandleNavArm = rclcpp_action::ClientGoalHandle<NavArmControl>;

  CallNavArmAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int64_t>("waypoint_index", 0, "Waypoint index for arm task"),
      BT::InputPort<int>("server_timeout", 1800000, "Timeout in ms"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavArmControl>::SharedPtr action_client_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;

  bool goal_completed_{false};
  int64_t result_success_{0};
  std::string result_message_;
  int64_t waypoint_index_{0};
  std::chrono::milliseconds server_timeout_{1800000};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace multi_action_node
