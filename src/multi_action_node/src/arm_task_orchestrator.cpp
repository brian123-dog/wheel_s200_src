#include "multi_action_node/arm_task_orchestrator.hpp"

#include "multi_action_node/arm_motion_executor.hpp"

#include <chrono>
#include <functional>
#include <cmath>
#include <string>
#include <thread>

namespace
{

geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

}  // namespace

ArmTaskOrchestrator::ArmTaskOrchestrator(std::shared_ptr<ArmMotionExecutor> executor)
: executor_(std::move(executor))
{
}

TaskRunResult ArmTaskOrchestrator::run_task_with_retries(
  int64_t waypoint_index,
  int task_max_retries,
  int step_max_retries,
  const FeedbackFn & feedback_cb)
{
  TaskRunResult final_result;
  final_result.ok = false;
  final_result.failed_stage = static_cast<int32_t>(TaskStage::kIdle);
  final_result.task_attempt_used = 0;
  final_result.message = "任务尚未开始";

  for (int task_attempt = 1; task_attempt <= task_max_retries; ++task_attempt) {
    const TaskRunResult one_attempt = run_single_task_attempt(
      waypoint_index, step_max_retries, task_attempt, feedback_cb);

    if (one_attempt.ok) {
      return one_attempt;
    }

    final_result = one_attempt;
    if (task_attempt < task_max_retries) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  return final_result;
}

TaskRunResult ArmTaskOrchestrator::run_single_task_attempt(
  int64_t waypoint_index,
  int step_max_retries,
  int task_attempt_index,
  const FeedbackFn & feedback_cb)
{
  // =========================================================================
  // 【简化版】不再执行复杂的杯子抓取-倾倒-放回流程。
  // 只做一个动作：将末端执行器绕 joint6 Z 轴旋转 10 度。
  // 原来的正向/逆向杯子流程（process_single_cup）全部保留在下方注释中，
  // 需要恢复时取消注释即可。
  // =========================================================================

  TaskRunResult result;
  result.ok = false;
  result.task_attempt_used = task_attempt_index;
  result.failed_stage = static_cast<int32_t>(TaskStage::kIdle);
  result.message = "未知失败";

  (void)waypoint_index;

  std::string err_msg;
  int32_t failed_stage = static_cast<int32_t>(TaskStage::kIdle);

  // --- 唯一动作：末端旋转 10 度 ---
  RCLCPP_INFO(rclcpp::get_logger("ArmTaskOrchestrator"),
    "执行简化任务: 末端旋转 10 度 (waypoint_index=%ld, attempt=%d)",
    waypoint_index, task_attempt_index);

  if (!run_step_with_retry(
      TaskStage::kPour,          // 复用 kPour stage 表示旋转动作
      step_max_retries,
      [this]() { return executor_->rotate_end_effector_by_angle(10.0); },
      feedback_cb,
      err_msg))
  {
    result.ok = false;
    result.failed_stage = failed_stage;
    result.message = "末端旋转10度失败: " + err_msg;
    return result;
  }

  // --- 任务成功 ---
  result.ok = true;
  result.failed_stage = static_cast<int32_t>(TaskStage::kDone);
  result.message = "末端旋转10度完成";
  if (feedback_cb) {
    feedback_cb(static_cast<double>(TaskStage::kDone));
  }

  return result;

  // =========================================================================
  // 【保留】原始杯子流程代码（已注释），需要时可恢复
  // =========================================================================
  // for (int i = 0; i < 5; ++i) {
  //   if (!process_single_cup(i, i, 5, step_max_retries, feedback_cb, failed_stage, err_msg)) {
  //     result.ok = false;
  //     result.failed_stage = failed_stage;
  //     result.message = "正向第 " + std::to_string(i + 1) + " 个杯子失败: " + err_msg;
  //     return result;
  //   }
  // }
  //
  // for (int i = 4; i >= 0; --i) {
  //   const int current_idx = 4 - i;
  //   if (!process_single_cup(i, current_idx, 5, step_max_retries, feedback_cb, failed_stage, err_msg)) {
  //     result.ok = false;
  //     result.failed_stage = failed_stage;
  //     result.message = "逆向第 " + std::to_string(current_idx + 1) + " 个杯子失败: " + err_msg;
  //     return result;
  //   }
  // }
  //
  // result.ok = true;
  // result.failed_stage = static_cast<int32_t>(TaskStage::kDone);
  // result.message = "正向与逆向任务全部完成";
  // if (feedback_cb) {
  //   feedback_cb(static_cast<double>(TaskStage::kDone));
  // }
  // return result;
}

bool ArmTaskOrchestrator::process_single_cup(
  int cup_id,
  int current_idx,
  int total_count,
  int step_max_retries,
  const FeedbackFn & feedback_cb,
  int32_t & failed_stage_out,
  std::string & err_msg_out)
{
  (void)current_idx;
  (void)total_count;

  geometry_msgs::msg::Pose pre_grasp;
  geometry_msgs::msg::Pose grasp;
  geometry_msgs::msg::Pose up_grasp;
  if (!executor_->get_cup_pose(cup_id, pre_grasp, grasp, up_grasp)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kPreGrasp);
    err_msg_out = "获取杯子位姿失败";
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kPreGrasp,
      step_max_retries,
      [this, &pre_grasp]() { return executor_->move_to_pose(pre_grasp); },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kPreGrasp);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kPreGrasp,
      step_max_retries,
      [this, cup_id]() {
        executor_->delete_cup_obstacle(cup_id);
        return true;
      },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kPreGrasp);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kGrasp,
      step_max_retries,
      [this]() { return executor_->control_gripper_open(); },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kGrasp);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kApproach,
      step_max_retries,
      [this, &grasp]() { return executor_->move_linear(grasp); },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kApproach);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kGrasp,
      step_max_retries,
      [this]() { return executor_->control_gripper_close(); },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kGrasp);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kLift,
      step_max_retries,
      [this, &up_grasp]() { return executor_->move_linear(up_grasp); },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kLift);
    return false;
  }

  geometry_msgs::msg::Pose mid_2_pose = up_grasp;
  mid_2_pose.position.x = -0.40;
  mid_2_pose.position.y = -0.50;
  mid_2_pose.orientation = rpy_to_quaternion(M_PI / 2.0, -M_PI / 2.0, 0.0);

  geometry_msgs::msg::Pose target_pose_1 = up_grasp;
  target_pose_1.position.x = -0.015;
  target_pose_1.position.y = -0.85;
  target_pose_1.orientation = rpy_to_quaternion(M_PI / 2.0, -M_PI / 2.0, 0.0);

  if (!run_step_with_retry(
      TaskStage::kTransfer,
      step_max_retries,
      [this, cup_id, &mid_2_pose, &target_pose_1]() {
        if (cup_id < 3) {
          return executor_->move_linear(target_pose_1);
        }
        return executor_->move_arc(mid_2_pose, target_pose_1, 50, true);
      },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kTransfer);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kPour,
      step_max_retries,
      [this]() { return executor_->rotate_end_effector(); },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kPour);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kReturn,
      step_max_retries,
      [this, cup_id, &mid_2_pose, &up_grasp]() {
        if (cup_id < 3) {
          return executor_->move_to_pose(up_grasp);
        }
        return executor_->move_arc(mid_2_pose, up_grasp, 50, false);
      },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kReturn);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kPlace,
      step_max_retries,
      [this, &grasp]() { return executor_->move_linear(grasp); },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kPlace);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kPlace,
      step_max_retries,
      [this]() { return executor_->control_gripper_open(); },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kPlace);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kRetreat,
      step_max_retries,
      [this, &pre_grasp]() { return executor_->move_linear(pre_grasp); },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kRetreat);
    return false;
  }

  if (!run_step_with_retry(
      TaskStage::kRetreat,
      step_max_retries,
      [this, cup_id]() {
        executor_->add_cup_obstacle(cup_id);
        return true;
      },
      feedback_cb,
      err_msg_out)) {
    failed_stage_out = static_cast<int32_t>(TaskStage::kRetreat);
    return false;
  }

  failed_stage_out = static_cast<int32_t>(TaskStage::kDone);
  err_msg_out.clear();
  return true;
}

bool ArmTaskOrchestrator::run_step_with_retry(
  TaskStage stage,
  int step_max_retries,
  const std::function<bool()> & step_fn,
  const FeedbackFn & feedback_cb,
  std::string & err_msg_out)
{
  if (feedback_cb) {
    feedback_cb(static_cast<double>(stage));
  }

  for (int attempt = 1; attempt <= step_max_retries; ++attempt) {
    if (step_fn()) {
      err_msg_out.clear();
      return true;
    }

    err_msg_out = "步骤失败，阶段=" + std::to_string(static_cast<int32_t>(stage)) +
      "，重试次数=" + std::to_string(attempt);

    if (attempt < step_max_retries) {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }

  return false;
}
