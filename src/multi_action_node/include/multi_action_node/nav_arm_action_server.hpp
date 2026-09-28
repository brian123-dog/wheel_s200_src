// nav_arm_action_server.hpp
#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_interfaces/action/nav_arm_control.hpp>

class ArmTaskOrchestrator;

class NavArmActionServer : public rclcpp::Node
{
public:
  using NavArmControl = base_interfaces::action::NavArmControl;
  using GoalHandleNavArm = rclcpp_action::ServerGoalHandle<NavArmControl>;

  explicit NavArmActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<NavArmControl>::SharedPtr action_server_;
  std::shared_ptr<ArmTaskOrchestrator> orchestrator_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavArmControl::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavArm> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNavArm> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavArm> goal_handle);
};