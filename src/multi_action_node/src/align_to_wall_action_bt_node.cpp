// align_to_wall_action_bt_node.cpp
// BT ActionNode implementation — calls /align_to_wall action and blocks.

#include "multi_action_node/align_to_wall_action_bt_node.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace multi_action_node
{

CallAlignToWall::CallAlignToWall(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus CallAlignToWall::onStart()
{
  double target_distance = 0.30;
  double target_yaw = 0.0;
  double tolerance_distance = 0.02;
  double tolerance_yaw = 0.017;
  std::string laser_frame = "laser_link";
  std::string camera_frame = "camera_link";
  int timeout_ms = 30000;

  getInput<double>("target_distance", target_distance);
  getInput<double>("target_yaw", target_yaw);
  getInput<double>("tolerance_distance", tolerance_distance);
  getInput<double>("tolerance_yaw", tolerance_yaw);
  getInput<std::string>("laser_frame", laser_frame);
  getInput<std::string>("camera_frame", camera_frame);
  getInput<int>("server_timeout", timeout_ms);

  server_timeout_ = std::chrono::milliseconds(timeout_ms);

  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("CallAlignToWall"),
      "[%s] No ROS node on blackboard", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  action_client_ = rclcpp_action::create_client<AlignToWall>(
    node_, "/align_to_wall", callback_group_);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
      "[%s] Action server '/align_to_wall' not available after 5s",
      name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto goal = AlignToWall::Goal();
  goal.target_distance = target_distance;
  goal.target_yaw = target_yaw;
  goal.tolerance_distance = tolerance_distance;
  goal.tolerance_yaw = tolerance_yaw;
  goal.laser_frame = laser_frame;
  goal.camera_frame = camera_frame;

  auto send_goal_options = rclcpp_action::Client<AlignToWall>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](auto future) {
      auto gh = future.get();
      if (!gh) {
        goal_completed_ = true;
        result_success_ = false;
        result_message_ = "Goal rejected";
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "[%s] Goal accepted", name().c_str());
    };
  send_goal_options.result_callback =
    [this](const GoalHandleAlign::WrappedResult & result) {
      goal_completed_ = true;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          result_success_ = result.result->success;
          result_message_ = result.result->message;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          result_success_ = false;
          result_message_ = result.result ? result.result->message : "Aborted";
          break;
        case rclcpp_action::ResultCode::CANCELED:
          result_success_ = false;
          result_message_ = "Cancelled";
          break;
        default:
          result_success_ = false;
          result_message_ = "Unknown";
          break;
      }
    };

  goal_completed_ = false;
  result_success_ = false;
  result_message_.clear();
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(),
    "[%s] Sending AlignToWall goal", name().c_str());

  action_client_->async_send_goal(goal, send_goal_options);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallAlignToWall::onRunning()
{
  callback_executor_.spin_some();

  if (goal_completed_) {
    if (result_success_) {
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
    result_success_ = false;
    result_message_ = "Timeout";
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void CallAlignToWall::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[%s] Halted", name().c_str());
}

}  // namespace multi_action_node
