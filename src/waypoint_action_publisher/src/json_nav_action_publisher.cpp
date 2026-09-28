#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
// #include "std_srvs/srv/empty.hpp"                              // [已注释] 机械臂服务相关
// #include "base_interfaces/srv/arm_task_execute.hpp"            // [已注释] 机械臂服务相关
#include <nlohmann/json.hpp>
#include "cabinet_log_summary.hpp"
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include <future>
#include <unistd.h>
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

class JsonWaypointNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Spin = nav2_msgs::action::Spin;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;
  using Trigger = std_srvs::srv::Trigger;
  // using Empty = std_srvs::srv::Empty;                           // [已注释] 机械臂服务相关
  // using ArmTaskExecute = base_interfaces::srv::ArmTaskExecute; // [已注释] 机械臂服务相关

  explicit JsonWaypointNavigator(const std::string& node_name = "json_waypoint_navigator")
  : Node(node_name)
  {
    // 声明参数
    waypoints_file_ = this->declare_parameter<std::string>(
      "waypoints_file", 
      "/home/sunrise/wheel_S200_src/src/waypoint_action_publisher/src/current_position.json"
    );
    
    frame_id_ = this->declare_parameter<std::string>(
      "frame_id", 
      "map"
    );
    
    wait_between_points_ = this->declare_parameter<bool>(
      "wait_between_points", 
      true
    );

    trigger_cabinet_task_ = this->declare_parameter<bool>(
      "trigger_cabinet_task",
      true
    );

    cabinet_service_name_ = this->declare_parameter<std::string>(
      "cabinet_service_name",
      "/rg_cabinet/run_sequence_from_qr"
    );

    cabinet_service_wait_timeout_sec_ = this->declare_parameter<int>(
      "cabinet_service_wait_timeout_sec",
      10
    );

    cabinet_service_call_timeout_sec_ = this->declare_parameter<int>(
      "cabinet_service_call_timeout_sec",
      1800
    );

    // [已注释] 机械臂服务相关参数
    /*
    connect_arm_ = this->declare_parameter<bool>(
      "connect_arm",
      true
    );

    arm_service_wait_timeout_sec_ = this->declare_parameter<int>(
      "arm_service_wait_timeout_sec",
      10
    );

    arm_service_call_timeout_sec_ = this->declare_parameter<int>(
      "arm_service_call_timeout_sec",
      1800
    );
    */

    position_tolerance_m_ = this->declare_parameter<double>(
      "position_tolerance_m",
      0.3
    );

    yaw_tolerance_rad_ = this->declare_parameter<double>(
      "yaw_tolerance_rad",
      0.35
    );

    enable_pose_refine_ = this->declare_parameter<bool>(
      "enable_pose_refine",
      true
    );

    refine_max_attempts_ = this->declare_parameter<int>(
      "refine_max_attempts",
      2
    );

    refine_timeout_sec_ = this->declare_parameter<int>(
      "refine_timeout_sec",
      30
    );

    refine_settle_time_sec_ = this->declare_parameter<double>(
      "refine_settle_time_sec",
      0.5
    );

    // [已注释] 原地转向精修相关参数
    /*
    enable_spin_refine_ = this->declare_parameter<bool>(
      "enable_spin_refine",
      true
    );

    spin_refine_timeout_sec_ = this->declare_parameter<int>(
      "spin_refine_timeout_sec",
      20
    );
    */

    // 创建导航动作客户端
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, 
      "navigate_to_pose"
    );
    // [已注释] 原地转向精修用 spin 客户端
    /*
    spin_client_ = rclcpp_action::create_client<Spin>(
      this,
      "spin"
    );
    */

    // TF 验证支持
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    cabinet_task_client_ = this->create_client<Trigger>(cabinet_service_name_);

    // [已注释] 创建机械臂触发服务客户端
    /*
    arm_task_client_ = this->create_client<Empty>("/rm_group/arm_task_execute");
    arm_task_detailed_client_ = this->create_client<ArmTaskExecute>("/rm_group/arm_task_execute_detailed");
    */

    // 加载航点
    if (!loadWaypoints()) {
      RCLCPP_ERROR(get_logger(), "加载航点失败，节点将退出");
      load_ok_ = false;
      return;
    }

    load_ok_ = true;

    RCLCPP_INFO(get_logger(), "JSON航点导航器已启动");
    RCLCPP_INFO(get_logger(), "加载了 %zu 个航点", waypoints_.size());
    RCLCPP_INFO(get_logger(), "航点间等待: %s", 
                wait_between_points_ ? "启用" : "禁用");
    RCLCPP_INFO(
      get_logger(),
      "QR柜体任务联动: %s, 服务=%s, 等待超时=%d秒, 调用超时=%d秒",
      trigger_cabinet_task_ ? "启用" : "禁用",
      cabinet_service_name_.c_str(),
      cabinet_service_wait_timeout_sec_,
      cabinet_service_call_timeout_sec_);
    // [已注释] 机械臂服务相关日志
    /*
    RCLCPP_INFO(get_logger(), "机械臂联动: %s",
          connect_arm_ ? "启用" : "禁用(仅导航)");
    RCLCPP_INFO(get_logger(), "机械臂服务等待超时: %d 秒, 调用超时: %d 秒",
          arm_service_wait_timeout_sec_, arm_service_call_timeout_sec_);
    */
    RCLCPP_INFO(get_logger(), "到点判定阈值: 距离<<=%.3f m, 角度<<=%.3f rad",
          position_tolerance_m_, yaw_tolerance_rad_);
    RCLCPP_INFO(get_logger(),
          "到点精修: %s (重试次数=%d, 单次超时=%d秒, 静稳等待=%.2f秒) —— 由每个航点的 enable_refine 控制是否执行",
          enable_pose_refine_ ? "可用" : "全局禁用",
          refine_max_attempts_, refine_timeout_sec_, refine_settle_time_sec_);
    // [已注释] 原地转向精修相关日志
    /*
    RCLCPP_INFO(get_logger(), "原地转向精修: %s, 超时=%d秒",
              enable_spin_refine_ ? "启用" : "禁用", spin_refine_timeout_sec_);
    */
  }

  bool isReady() const
  {
    return load_ok_;
  }

  void runNavigation()
  {
    if (!waitForNavServer()) {
      RCLCPP_ERROR(get_logger(), "导航服务器不可用");
      return;
    }

    RCLCPP_INFO(get_logger(), "开始执行航点导航...");

    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& wp = waypoints_[i];
      const double target_yaw = quaternionToYaw(wp.pose.pose.orientation);
      const auto waypoint_start_time = now();
      const double progress = (static_cast<double>(i + 1) / static_cast<double>(waypoints_.size())) * 100.0;
      
      RCLCPP_INFO(get_logger(), 
                  "[%.0f%%] 开始导航到航点 %zu/%zu: %s | 目标(x=%.3f, y=%.3f, z=%.3f, yaw=%.3f) | execute_tasks=%s, enable_refine=%s", 
                  progress,
                  i + 1, waypoints_.size(),
                  wp.name.c_str(),
                  wp.pose.pose.position.x,
                  wp.pose.pose.position.y,
                  wp.pose.pose.position.z,
                  target_yaw,
                  wp.execute_tasks ? "true" : "false",
                  wp.enable_refine ? "true" : "false");

      // 导航到当前航点
      // execute_tasks : 控制到站任务（QR柜体等）
      // enable_refine : 控制到点精修（两者独立，互不影响）
      if (!navigateToPose(wp.pose, wp.name, i + 1, waypoints_.size(), wp.enable_refine)) {
        RCLCPP_ERROR(get_logger(), "航点 %zu/%zu (%s) 导航失败", i + 1, waypoints_.size(), wp.name.c_str());

        // [已注释] 导航失败后触发机械臂任务
        /*
        if (!triggerArmTask(i + 1, waypoints_.size(), wp.name, wp.pose)) {
          RCLCPP_ERROR(get_logger(), "航点失败后触发机械臂任务也失败，终止后续航点执行");
        }
        */

        return;
      }

      const double elapsed_sec = (now() - waypoint_start_time).seconds();
      RCLCPP_INFO(get_logger(), "航点 %zu/%zu (%s) 到达成功，耗时 %.2f 秒", i + 1, waypoints_.size(), wp.name.c_str(), elapsed_sec);

      // --- 根据 execute_tasks 决定是否触发到站任务 ---
      if (wp.execute_tasks) {
        RCLCPP_INFO(get_logger(), "航点 %zu/%zu (%s): execute_tasks=true，触发到站任务", i + 1, waypoints_.size(), wp.name.c_str());
        if (!triggerCabinetTask(i + 1, waypoints_.size(), wp.name, wp.pose)) {
          RCLCPP_ERROR(get_logger(), "QR柜体任务失败，终止后续航点执行");
          return;
        }
      } else {
        RCLCPP_INFO(get_logger(), "航点 %zu/%zu (%s): execute_tasks=false，跳过到站任务", i + 1, waypoints_.size(), wp.name.c_str());
      }

      // 如果不是最后一个航点，等待指定时间
      if (i + 1 < waypoints_.size() && wait_between_points_) {
        double wait_time = wp.wait_time;
        if (wait_time > 0) {
          RCLCPP_INFO(get_logger(), 
                      "等待 %.1f 秒后前往下一个航点", 
                      wait_time);
          std::this_thread::sleep_for(
            std::chrono::duration<double>(wait_time)
          );
        }
      }
    }

    RCLCPP_INFO(get_logger(), "所有航点导航完成！");
  }

private:
  struct Waypoint {
    std::string name;
    geometry_msgs::msg::PoseStamped pose;
    double wait_time;
    bool execute_tasks = false;  // 到站任务标志（QR柜体/扫码/机械臂）
    bool enable_refine = false;  // 到点精修标志（与 execute_tasks 独立）
  };

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<Trigger>::SharedPtr cabinet_task_client_;
  // [已注释] 原地转向精修用 spin 客户端
  /*
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  */
  // [已注释] 机械臂服务客户端
  /*
  rclcpp::Client<Empty>::SharedPtr arm_task_client_;
  rclcpp::Client<ArmTaskExecute>::SharedPtr arm_task_detailed_client_;
  */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<Waypoint> waypoints_;
  std::string waypoints_file_;
  std::string frame_id_;
  bool wait_between_points_;
  bool trigger_cabinet_task_;
  std::string cabinet_service_name_;
  int cabinet_service_wait_timeout_sec_;
  int cabinet_service_call_timeout_sec_;
  // [已注释] 机械臂相关参数
  /*
  bool connect_arm_;
  int arm_service_wait_timeout_sec_;
  int arm_service_call_timeout_sec_;
  */
  double position_tolerance_m_;
  double yaw_tolerance_rad_;
  bool enable_pose_refine_;
  int refine_max_attempts_;
  int refine_timeout_sec_;
  double refine_settle_time_sec_;
  // [已注释] 原地转向精修相关参数
  /*
  bool enable_spin_refine_;
  int spin_refine_timeout_sec_;
  */
  bool load_ok_ = false;

  double normalizeAngle(double angle) const
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  bool isPositionReached(const geometry_msgs::msg::PoseStamped& target,
                         double position_threshold,
                         double yaw_threshold)
  {
    try {
      auto transform = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));

      double dx = target.pose.position.x - transform.transform.translation.x;
      double dy = target.pose.position.y - transform.transform.translation.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      const double target_yaw = quaternionToYaw(target.pose.orientation);
      const double current_yaw = tf2::getYaw(transform.transform.rotation);
      const double yaw_error = std::abs(normalizeAngle(target_yaw - current_yaw));

      RCLCPP_INFO(
        get_logger(),
        "位置姿态验证: 目标(%.2f,%.2f,yaw=%.2f) vs 当前(%.2f,%.2f,yaw=%.2f), 距离=%.2f米, 角误差=%.2f弧度",
        target.pose.position.x,
        target.pose.position.y,
        target_yaw,
        transform.transform.translation.x,
        transform.transform.translation.y,
        current_yaw,
        dist,
        yaw_error);

      return dist <= position_threshold && yaw_error <= yaw_threshold;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "TF查询失败: %s", e.what());
      return false;
    }
  }

  bool getPoseError(const geometry_msgs::msg::PoseStamped& target,
                    double& dist,
                    double& yaw_error,
                    double& signed_yaw_error)
  {
    try {
      auto transform = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));

      const double dx = target.pose.position.x - transform.transform.translation.x;
      const double dy = target.pose.position.y - transform.transform.translation.y;
      dist = std::sqrt(dx * dx + dy * dy);

      const double target_yaw = quaternionToYaw(target.pose.orientation);
      const double current_yaw = tf2::getYaw(transform.transform.rotation);
      signed_yaw_error = normalizeAngle(target_yaw - current_yaw);
      yaw_error = std::abs(signed_yaw_error);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "姿态误差计算失败: %s", e.what());
      return false;
    }
  }

  // [已注释] 原地转向精修函数
  /*
  bool spinRefineToTargetYaw(const geometry_msgs::msg::PoseStamped& target)
  {
    if (!enable_spin_refine_) {
      return false;
    }

    if (!spin_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(get_logger(), "spin 动作服务器不可用，跳过原地转向精修");
      return false;
    }

    double dist = 0.0;
    double yaw_error = 0.0;
    double signed_yaw_error = 0.0;
    if (!getPoseError(target, dist, yaw_error, signed_yaw_error)) {
      return false;
    }

    if (yaw_error <= yaw_tolerance_rad_) {
      return true;
    }

    Spin::Goal spin_goal;
    spin_goal.target_yaw = static_cast<float>(signed_yaw_error);
    spin_goal.time_allowance = builtin_interfaces::msg::Duration();
    spin_goal.time_allowance.sec = spin_refine_timeout_sec_;
    spin_goal.time_allowance.nanosec = 0;

    RCLCPP_INFO(
      get_logger(),
      "触发原地转向精修: 当前角误差=%.3f rad, 旋转目标=%.3f rad",
      yaw_error,
      signed_yaw_error);

    auto send_goal_future = spin_client_->async_send_goal(spin_goal);
    if (rclcpp::spin_until_future_complete(
          shared_from_this(),
          send_goal_future,
          5s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "原地转向精修发送目标失败");
      return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(get_logger(), "原地转向精修目标被拒绝");
      return false;
    }

    auto result_future = spin_client_->async_get_result(goal_handle);
    auto status = rclcpp::spin_until_future_complete(
      shared_from_this(),
      result_future,
      std::chrono::seconds(spin_refine_timeout_sec_));

    if (status == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_WARN(get_logger(), "原地转向精修超时(%d秒)", spin_refine_timeout_sec_);
      spin_client_->async_cancel_goal(goal_handle);
      return false;
    }
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "原地转向精修结果获取失败");
      return false;
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(get_logger(), "原地转向精修执行失败: %s", resultCodeToString(result.code));
      return false;
    }

    RCLCPP_INFO(get_logger(), "原地转向精修执行完成");
    return true;
  }
  */

  // [已注释] 机械臂任务触发函数
  /*
  bool triggerArmTask(
    size_t waypoint_index,
    size_t waypoint_total,
    const std::string& waypoint_name,
    const geometry_msgs::msg::PoseStamped& expected_pose)
  {
    if (!connect_arm_) {
      RCLCPP_INFO(
        get_logger(),
        "航点 %zu/%zu (%s) 已到达，connect_arm=false，跳过机械臂任务",
        waypoint_index,
        waypoint_total,
        waypoint_name.c_str());
      return true;
    }

    // 触发前再次确认位置，避免等待服务期间被碰移后误触发机械臂
    if (!isPositionReached(expected_pose, position_tolerance_m_, yaw_tolerance_rad_)) {
      RCLCPP_ERROR(get_logger(), "机械臂触发前位置或姿态偏离，放弃触发");
      return false;
    }

    const auto service_wait_timeout = std::chrono::seconds(arm_service_wait_timeout_sec_);
    const auto service_call_timeout = std::chrono::seconds(60);

    // 优先走详细服务：只有 arm_task_done=true 且 success=true 才继续下一航点
    if (arm_task_detailed_client_->wait_for_service(service_wait_timeout)) {
      auto request = std::make_shared<ArmTaskExecute::Request>();
      request->waypoint_index = static_cast<int64_t>(waypoint_index);
      request->run_full_task = false;

      RCLCPP_INFO(get_logger(), "[调试] 准备调用机械臂服务...");
      auto future = arm_task_detailed_client_->async_send_request(request);
      RCLCPP_INFO(get_logger(), "[调试] 已发送请求，等待响应（最多60秒）...");
      RCLCPP_INFO(
        get_logger(),
        "航点 %zu/%zu (%s) 到达，触发机械臂详细任务...",
        waypoint_index,
        waypoint_total,
        waypoint_name.c_str());

      auto status = rclcpp::spin_until_future_complete(shared_from_this(), future, service_call_timeout);
      if (status == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "❌ 机械臂服务调用超时60秒，强制继续");
        return false;
      }
      if (status != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
          get_logger(),
          "航点 %zu/%zu (%s) 机械臂详细任务调用失败或超时",
          waypoint_index,
          waypoint_total,
          waypoint_name.c_str());
        return false;
      }

      auto result = future.get();
      if (!result) {
        RCLCPP_ERROR(get_logger(), "机械臂详细任务返回空响应");
        return false;
      }
      if (!result->arm_task_done || !result->success) {
        RCLCPP_ERROR(
          get_logger(),
          "航点 %zu/%zu (%s) 机械臂任务未完成或失败: done=%s success=%s stage=%d msg=%s",
          waypoint_index,
          waypoint_total,
          waypoint_name.c_str(),
          result->arm_task_done ? "true" : "false",
          result->success ? "true" : "false",
          result->failed_stage,
          result->message.c_str());
        return false;
      }

      RCLCPP_INFO(
        get_logger(),
        "航点 %zu/%zu (%s) 机械臂任务完成(task_id=%d)，继续后续航点",
        waypoint_index,
        waypoint_total,
        waypoint_name.c_str(),
        result->task_id);
      return true;
    }

    if (!arm_task_client_->wait_for_service(service_wait_timeout)) {
      RCLCPP_ERROR(
        get_logger(),
        "航点 %zu/%zu (%s) 到达后，机械臂服务不可用（详细/基础均等待超时）",
        waypoint_index,
        waypoint_total,
        waypoint_name.c_str());
      return false;
    }

    auto request = std::make_shared<Empty::Request>();
    RCLCPP_INFO(get_logger(), "[调试] 准备调用机械臂服务...");
    auto future = arm_task_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "[调试] 已发送请求，等待响应（最多60秒）...");

    RCLCPP_INFO(
      get_logger(),
      "航点 %zu/%zu (%s) 到达，触发机械臂任务...",
      waypoint_index,
      waypoint_total,
      waypoint_name.c_str());

    auto status = rclcpp::spin_until_future_complete(shared_from_this(), future, service_call_timeout);
    if (status == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(get_logger(), "❌ 机械臂服务调用超时60秒，强制继续");
      return false;
    }
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "航点 %zu/%zu (%s) 机械臂任务调用失败或超时",
        waypoint_index,
        waypoint_total,
        waypoint_name.c_str());
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "航点 %zu/%zu (%s) 机械臂任务完成，继续后续航点",
      waypoint_index,
      waypoint_total,
      waypoint_name.c_str());
    return true;
  }
  */

  bool triggerCabinetTask(
    size_t waypoint_index,
    size_t waypoint_total,
    const std::string& waypoint_name,
    const geometry_msgs::msg::PoseStamped& expected_pose)
  {
    if (!trigger_cabinet_task_) {
      RCLCPP_INFO(
        get_logger(),
        "航点 %zu/%zu (%s) 已到达，trigger_cabinet_task=false，跳过QR柜体任务",
        waypoint_index,
        waypoint_total,
        waypoint_name.c_str());
      return true;
    }

    // 触发前再次确认底盘仍在目标附近，避免等待服务期间被移动后误操作电柜。
    if (!isPositionReached(expected_pose, position_tolerance_m_, yaw_tolerance_rad_)) {
      RCLCPP_ERROR(get_logger(), "QR柜体任务触发前底盘位姿偏离，放弃触发");
      return false;
    }

    const int wait_timeout_sec =
      cabinet_service_wait_timeout_sec_ > 0 ? cabinet_service_wait_timeout_sec_ : 1;
    const int call_timeout_sec =
      cabinet_service_call_timeout_sec_ > 0 ? cabinet_service_call_timeout_sec_ : 1;
    const auto service_wait_timeout = std::chrono::seconds(wait_timeout_sec);
    const auto service_call_timeout = std::chrono::seconds(call_timeout_sec);

    if (!cabinet_task_client_->wait_for_service(service_wait_timeout)) {
      RCLCPP_ERROR(
        get_logger(),
        "航点 %zu/%zu (%s) 到达后，QR柜体任务服务不可用: %s",
        waypoint_index,
        waypoint_total,
        waypoint_name.c_str(),
        cabinet_service_name_.c_str());
      return false;
    }

    auto request = std::make_shared<Trigger::Request>();
    RCLCPP_INFO(
      get_logger(),
      "航点 %zu/%zu (%s) 到达，调用QR柜体任务服务: %s",
      waypoint_index,
      waypoint_total,
      waypoint_name.c_str(),
      cabinet_service_name_.c_str());

    auto future = cabinet_task_client_->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(
      shared_from_this(),
      future,
      service_call_timeout);

    if (status == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(
        get_logger(),
        "QR柜体任务服务调用超时%d秒: %s",
        call_timeout_sec,
        cabinet_service_name_.c_str());
      return false;
    }
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(),
        "QR柜体任务服务调用失败: %s",
        cabinet_service_name_.c_str());
      return false;
    }

    auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(get_logger(), "QR柜体任务服务返回空响应");
      return false;
    }
    if (!response->success) {
      RCLCPP_ERROR(
        get_logger(),
        "QR柜体任务执行失败: %s",
        cabinet_response_summary(response->message).c_str());
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "QR柜体任务执行完成: %s",
      cabinet_response_summary(response->message).c_str());
    return true;
  }

  bool loadWaypoints()
  {
    waypoints_.clear();

    // 直接使用参数指定的绝对路径
    const std::string full_path = waypoints_file_;
    
    RCLCPP_INFO(get_logger(), 
                "尝试加载航点文件: %s", 
                full_path.c_str());

    std::ifstream file(full_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), 
                   "无法打开航点文件: %s", 
                   full_path.c_str());
      return false;
    }

    try {
      json data = json::parse(file);

      // 格式1：waypoints数组
      if (data.contains("waypoints") && data["waypoints"].is_array()) {
        for (const auto& wp_json : data["waypoints"]) {
          Waypoint wp;

          wp.name = wp_json.value("name", "unnamed");
          wp.pose.header.frame_id = frame_id_;
          wp.pose.pose.position.x = wp_json["x"].get<double>();
          wp.pose.pose.position.y = wp_json["y"].get<double>();
          wp.pose.pose.position.z = wp_json.value("z", 0.0);

          double yaw = wp_json.value("yaw", 0.0);
          wp.pose.pose.orientation = eulerToQuaternion(0.0, 0.0, yaw);

          wp.wait_time = wp_json.value("wait_time", 3.0);
          wp.execute_tasks = wp_json.value("execute_tasks", false);  // 到站任务标志
          wp.enable_refine = wp_json.value("enable_refine", false);  // 到点精修标志
          waypoints_.push_back(wp);
        }
      }
      // 格式2：position（支持对象或对象数组）
      else if (data.contains("position")) {
        const auto parse_position_obj = [this](const json& pos, size_t index) {
          Waypoint wp;
          wp.name = "position_" + std::to_string(index + 1);
          wp.pose.header.frame_id = frame_id_;
          wp.pose.pose.position.x = pos["x"].get<double>();
          wp.pose.pose.position.y = pos["y"].get<double>();
          wp.pose.pose.position.z = pos.value("z", 0.0);

          const double yaw = pos.value("yaw", 0.0);
          wp.pose.pose.orientation = eulerToQuaternion(0.0, 0.0, yaw);
          wp.wait_time = pos.value("wait_time", 3.0);
          wp.execute_tasks = pos.value("execute_tasks", false);  // 到站任务标志
          wp.enable_refine = pos.value("enable_refine", false);  // 到点精修标志
          waypoints_.push_back(wp);
        };

        if (data["position"].is_object()) {
          parse_position_obj(data["position"], 0);
        } else if (data["position"].is_array()) {
          size_t index = 0;
          for (const auto& pos : data["position"]) {
            if (!pos.is_object()) {
              RCLCPP_ERROR(get_logger(), "position[%zu] 不是对象", index);
              return false;
            }
            parse_position_obj(pos, index);
            ++index;
          }
        } else {
          RCLCPP_ERROR(get_logger(), "JSON文件格式错误: 'position' 必须是对象或数组");
          return false;
        }
      } else {
        RCLCPP_ERROR(get_logger(),
                     "JSON文件格式错误: 需要 'waypoints' 数组或 'position' 对象");
        return false;
      }

      if (waypoints_.empty()) {
        RCLCPP_ERROR(get_logger(), "未解析到任何可用航点");
        return false;
      }

      RCLCPP_INFO(get_logger(), 
                  "成功加载 %zu 个航点", 
                  waypoints_.size());
      return true;

    } catch (const json::exception& e) {
      RCLCPP_ERROR(get_logger(), 
                   "JSON解析错误: %s", 
                   e.what());
      return false;
    }
  }

  geometry_msgs::msg::Quaternion eulerToQuaternion(double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
  }

  bool waitForNavServer()
  {
    RCLCPP_INFO(get_logger(), 
                "等待导航服务器...");
    
    if (!nav_client_->wait_for_action_server(30s)) {
      RCLCPP_ERROR(get_logger(), 
                   "导航服务器在30秒内不可用");
      return false;
    }

    RCLCPP_INFO(get_logger(), 
                "导航服务器已就绪");
    return true;
  }

  double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) const
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  const char* resultCodeToString(rclcpp_action::ResultCode code) const
  {
    switch (code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return "SUCCEEDED";
      case rclcpp_action::ResultCode::ABORTED:
        return "ABORTED";
      case rclcpp_action::ResultCode::CANCELED:
        return "CANCELED";
      default:
        return "UNKNOWN";
    }
  }

  bool navigateToPose(const geometry_msgs::msg::PoseStamped& pose,
                      const std::string& waypoint_name,
                      size_t waypoint_index,
                      size_t waypoint_total,
                      bool enable_refine)
  {
    const auto nav_start_time = now();

    // 更新时间戳
    auto goal_pose = pose;
    goal_pose.header.stamp = now();

    auto goal = NavigateToPose::Goal();
    goal.pose = goal_pose;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    // 设置目标响应回调 - 修复回调签名
    options.goal_response_callback = 
      [this](GoalHandleNav::SharedPtr goal_handle) {   // ✅ 正确的参数类型
        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "目标被服务器拒绝");
        } else {
          RCLCPP_INFO(get_logger(), "目标已被接受");
        }
      };

    options.feedback_callback =
      [this, waypoint_name, waypoint_index, waypoint_total](
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        const double eta_sec = rclcpp::Duration(feedback->estimated_time_remaining).seconds();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "航点 %zu/%zu (%s) 进行中: 剩余距离=%.2f m, 预计剩余=%.1f s, 恢复次数=%d",
          waypoint_index,
          waypoint_total,
          waypoint_name.c_str(),
          feedback->distance_remaining,
          eta_sec,
          feedback->number_of_recoveries);
      };

    RCLCPP_INFO(
      get_logger(),
      "发送导航目标: 航点 %zu/%zu (%s), frame=%s, x=%.3f, y=%.3f, z=%.3f",
      waypoint_index,
      waypoint_total,
      waypoint_name.c_str(),
      goal_pose.header.frame_id.c_str(),
      goal_pose.pose.position.x,
      goal_pose.pose.position.y,
      goal_pose.pose.position.z);

    // 发送目标
    auto send_goal_future = nav_client_->async_send_goal(goal, options);
    
    // 等待发送完成
    if (rclcpp::spin_until_future_complete(
          shared_from_this(), 
          send_goal_future, 
          10s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "发送目标失败");
      return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "目标被拒绝");
      return false;
    }

    RCLCPP_INFO(get_logger(), "已接受航点 %zu/%zu，开始独立监控...", waypoint_index, waypoint_total);

    auto result_future = nav_client_->async_get_result(goal_handle);

    // 设定一个合理的超时时间（例如 120 秒），防止永久阻塞
    const auto timeout = 120s;
    auto status = rclcpp::spin_until_future_complete(
        shared_from_this(), result_future, timeout);

    if (status == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "导航超时（%ld 秒），取消目标", timeout.count());
        nav_client_->async_cancel_goal(goal_handle);
        // 给取消操作一点时间
        std::this_thread::sleep_for(500ms);
        return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        const double nav_elapsed_sec = (now() - nav_start_time).seconds();
        RCLCPP_INFO(get_logger(), "✅ Nav2 报告导航成功，耗时 %.2f 秒", nav_elapsed_sec);

        if (refine_settle_time_sec_ > 0.0) {
          std::this_thread::sleep_for(std::chrono::duration<double>(refine_settle_time_sec_));
        }

        if (!isPositionReached(goal_pose, position_tolerance_m_, yaw_tolerance_rad_)) {
          // ---- enable_refine=false: 本航点不做精修，导航到点即视为完成 ----
          if (!enable_refine) {
            RCLCPP_INFO(
              get_logger(),
              "航点 %zu/%zu (%s): enable_refine=false，跳过到点精修（位姿容差仅记录不强制）",
              waypoint_index,
              waypoint_total,
              waypoint_name.c_str());
            return true;
          }

          // [已注释] 原地转向精修相关代码块
          /*
          if (enable_spin_refine_) {
            spinRefineToTargetYaw(goal_pose);
            if (refine_settle_time_sec_ > 0.0) {
              std::this_thread::sleep_for(std::chrono::duration<double>(refine_settle_time_sec_));
            }
            if (isPositionReached(goal_pose, position_tolerance_m_, yaw_tolerance_rad_)) {
              RCLCPP_INFO(get_logger(), "原地转向精修后位姿满足阈值");
              return true;
            }
          }
          */

          if (!enable_pose_refine_ || refine_max_attempts_ <= 0) {
            RCLCPP_ERROR(
              get_logger(),
              "Nav2返回成功但最终位姿未满足阈值(距离<<=%.3f m, 角度<<=%.3f rad)，且精修已禁用",
              position_tolerance_m_,
              yaw_tolerance_rad_);
            return false;
          }

          RCLCPP_WARN(
            get_logger(),
            "航点 %zu/%zu (%s): enable_refine=true，Nav2返回成功但位姿未满足阈值，开始到点精修(最多%d次)",
            waypoint_index,
            waypoint_total,
            waypoint_name.c_str(),
            refine_max_attempts_);

          for (int attempt = 1; attempt <= refine_max_attempts_; ++attempt) {
            auto refine_pose = goal_pose;
            refine_pose.header.stamp = now();
            NavigateToPose::Goal refine_goal;
            refine_goal.pose = refine_pose;

            RCLCPP_INFO(
              get_logger(),
              "到点精修 %d/%d: 重新发送同一目标",
              attempt,
              refine_max_attempts_);

            auto refine_send_goal_future = nav_client_->async_send_goal(refine_goal);
            if (rclcpp::spin_until_future_complete(
                  shared_from_this(),
                  refine_send_goal_future,
                  10s) != rclcpp::FutureReturnCode::SUCCESS) {
              RCLCPP_WARN(get_logger(), "到点精修 %d/%d: 发送目标失败", attempt, refine_max_attempts_);
              continue;
            }

            auto refine_goal_handle = refine_send_goal_future.get();
            if (!refine_goal_handle) {
              RCLCPP_WARN(get_logger(), "到点精修 %d/%d: 目标被拒绝", attempt, refine_max_attempts_);
              continue;
            }

            auto refine_result_future = nav_client_->async_get_result(refine_goal_handle);
            auto refine_status = rclcpp::spin_until_future_complete(
              shared_from_this(),
              refine_result_future,
              std::chrono::seconds(refine_timeout_sec_));

            if (refine_status == rclcpp::FutureReturnCode::TIMEOUT) {
              RCLCPP_WARN(
                get_logger(),
                "到点精修 %d/%d: 超时(%d秒)，取消该次目标",
                attempt,
                refine_max_attempts_,
                refine_timeout_sec_);
              nav_client_->async_cancel_goal(refine_goal_handle);
              continue;
            }
            if (refine_status != rclcpp::FutureReturnCode::SUCCESS) {
              RCLCPP_WARN(get_logger(), "到点精修 %d/%d: 获取结果失败", attempt, refine_max_attempts_);
              continue;
            }

            auto refine_result = refine_result_future.get();
            if (refine_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_WARN(
                get_logger(),
                "到点精修 %d/%d: Nav2结果=%s",
                attempt,
                refine_max_attempts_,
                resultCodeToString(refine_result.code));
              continue;
            }

            if (refine_settle_time_sec_ > 0.0) {
              std::this_thread::sleep_for(std::chrono::duration<double>(refine_settle_time_sec_));
            }

            if (isPositionReached(refine_pose, position_tolerance_m_, yaw_tolerance_rad_)) {
              RCLCPP_INFO(get_logger(), "到点精修成功，最终位姿满足阈值");
              return true;
            }

            RCLCPP_WARN(
              get_logger(),
              "到点精修 %d/%d 后仍未满足阈值，继续下一次",
              attempt,
              refine_max_attempts_);
          }

          RCLCPP_ERROR(
            get_logger(),
            "Nav2返回成功，但经过%d次到点精修后仍未满足阈值(距离<<=%.3f m, 角度<<=%.3f rad)",
            refine_max_attempts_,
            position_tolerance_m_,
            yaw_tolerance_rad_);
          return false;
        }

        return true;
    } else {
        RCLCPP_ERROR(get_logger(), "Nav2 报告失败: %s", resultCodeToString(result.code));
        return false;
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // [修复1] 添加唯一后缀，避免节点名冲突
  auto node_name = "json_waypoint_navigator_" + 
                   std::to_string(getpid()) + "_" + 
                   std::to_string(rclcpp::Clock().now().nanoseconds() % 10000);
  
  auto node = std::make_shared<JsonWaypointNavigator>(node_name);

  if (!node->isReady()) {
    rclcpp::shutdown();
    return 1;
  }
  
  std::this_thread::sleep_for(2s);
  node->runNavigation();
  
  // [修复2] 显式清理节点再shutdown
  node.reset();  // 强制析构，释放Action Client资源
  std::this_thread::sleep_for(500ms);  // 给DDS时间传播
  
  rclcpp::shutdown();
  return 0;
}
