// nav_arm_action_bt_node.cpp
// BT ActionNode implementation that calls /nav_arm_control action and blocks.

#include "multi_action_node/nav_arm_action_bt_node.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace multi_action_node
{

CallNavArmAction::CallNavArmAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus CallNavArmAction::onStart()
{
  if (!getInput<int64_t>("waypoint_index", waypoint_index_)) {
    RCLCPP_ERROR(rclcpp::get_logger("CallNavArmAction"),
      "[%s] Missing required input [waypoint_index]", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  int timeout_ms = 1800000;
  getInput<int>("server_timeout", timeout_ms);
  server_timeout_ = std::chrono::milliseconds(timeout_ms);

  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("CallNavArmAction"),
      "[%s] No ROS node on blackboard", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  action_client_ = rclcpp_action::create_client<NavArmControl>(
    node_, "nav_arm_control", callback_group_);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
      "[%s] Action server 'nav_arm_control' not available", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto goal = NavArmControl::Goal();
  goal.waypoint_index = waypoint_index_;

  auto send_goal_options = rclcpp_action::Client<NavArmControl>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](auto future) {
      auto gh = future.get();
      if (!gh) {
        goal_completed_ = true;
        result_success_ = 0;
        result_message_ = "Goal rejected";
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "[%s] Goal accepted", name().c_str());
    };
  send_goal_options.result_callback =
    [this](const GoalHandleNavArm::WrappedResult & result) {
      goal_completed_ = true;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          result_success_ = result.result->success;
          result_message_ = result.result->message;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          result_success_ = 0;
          result_message_ = result.result ? result.result->message : "Aborted";
          break;
        case rclcpp_action::ResultCode::CANCELED:
          result_success_ = 0;
          result_message_ = "Cancelled";
          break;
        default:
          result_success_ = 0;
          result_message_ = "Unknown";
          break;
      }
    };

  goal_completed_ = false;
  result_success_ = 0;
  result_message_.clear();
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(),
    "[%s] Sending NavArmControl goal (waypoint_index=%ld)",
    name().c_str(), waypoint_index_);

  action_client_->async_send_goal(goal, send_goal_options);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallNavArmAction::onRunning()
{
  callback_executor_.spin_some();

  if (goal_completed_) {
    if (result_success_ == 1) {
      RCLCPP_INFO(node_->get_logger(), "[%s] SUCCESS: %s",
        name().c_str(), result_message_.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[%s] FAILED: %s",
        name().c_str(), result_message_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > server_timeout_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] TIMEOUT", name().c_str());
    goal_completed_ = true;
    result_success_ = 0;
    result_message_ = "Timeout";
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void CallNavArmAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[%s] Halted", name().c_str());
}

}  // namespace multi_action_node

// #include "behaviortree_cpp_v3/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<multi_action_node::CallNavArmAction>("CallNavArmAction");
// }
