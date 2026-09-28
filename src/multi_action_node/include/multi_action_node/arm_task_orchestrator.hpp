// arm_task_orchestrator.hpp
#pragma once

#include <memory>
#include <functional>
#include <base_interfaces/action/nav_arm_control.hpp>
#include <geometry_msgs/msg/pose.hpp>

class ArmMotionExecutor;

enum class TaskStage : int32_t
{
  kIdle = 0,
  kPreGrasp = 10,
  kApproach = 20,
  kGrasp = 30,
  kLift = 40,
  kTransfer = 50,
  kPour = 60,
  kReturn = 70,
  kPlace = 80,
  kRetreat = 90,
  kDone = 100
};

struct TaskRunResult
{
  bool ok {false};
  int32_t failed_stage {0};
  int32_t task_attempt_used {0};
  std::string message;
};

class ArmTaskOrchestrator
{
public:
  using FeedbackFn = std::function<void(double)>;

  explicit ArmTaskOrchestrator(std::shared_ptr<ArmMotionExecutor> executor);

  TaskRunResult run_task_with_retries(
    int64_t waypoint_index,
    int task_max_retries,
    int step_max_retries,
    const FeedbackFn & feedback_cb);

private:
  std::shared_ptr<ArmMotionExecutor> executor_;

  TaskRunResult run_single_task_attempt(
    int64_t waypoint_index,
    int step_max_retries,
    int task_attempt_index,
    const FeedbackFn & feedback_cb);

  bool process_single_cup(
    int cup_id,
    int current_idx,
    int total_count,
    int step_max_retries,
    const FeedbackFn & feedback_cb,
    int32_t & failed_stage_out,
    std::string & err_msg_out);

  bool run_step_with_retry(
    TaskStage stage,
    int step_max_retries,
    const std::function<bool()> & step_fn,
    const FeedbackFn & feedback_cb,
    std::string & err_msg_out);
};