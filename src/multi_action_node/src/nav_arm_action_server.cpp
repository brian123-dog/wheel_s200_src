#include "multi_action_node/nav_arm_action_server.hpp"
#include "multi_action_node/arm_motion_executor.hpp"
#include "multi_action_node/arm_task_orchestrator.hpp"

#include <memory>
#include <stdexcept>
#include <thread>

NavArmActionServer::NavArmActionServer(const rclcpp::NodeOptions & options)
: Node("nav_arm_action_server", options)
{
    using namespace std::placeholders;

    // 创建执行器和编排器
    auto executor = std::make_shared<ArmMotionExecutor>(this);
    if (!executor->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "ArmMotionExecutor 初始化失败");
        throw std::runtime_error("ArmMotionExecutor 初始化失败");
    }
    
    orchestrator_ = std::make_shared<ArmTaskOrchestrator>(executor);

    // 创建Action Server
    this->action_server_ = rclcpp_action::create_server<NavArmControl>(
        this,
        "nav_arm_control",
        std::bind(&NavArmActionServer::handle_goal, this, _1, _2),
        std::bind(&NavArmActionServer::handle_cancel, this, _1),
        std::bind(&NavArmActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "NavArm 动作服务已启动。");
}

rclcpp_action::GoalResponse NavArmActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavArmControl::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "收到目标请求，waypoint_index: %ld", goal->waypoint_index);
    (void)uuid;
    
    // 校验目标合法性
    if (goal->waypoint_index < 0) {
        RCLCPP_WARN(this->get_logger(), "无效的 waypoint_index: %ld", goal->waypoint_index);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 可以添加更多校验规则
    if (goal->waypoint_index > 10) { // 假设最多10个航点
        RCLCPP_WARN(this->get_logger(), "waypoint_index 超出范围: %ld", goal->waypoint_index);
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavArmActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleNavArm> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "收到取消目标请求");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavArmActionServer::handle_accepted(const std::shared_ptr<GoalHandleNavArm> goal_handle)
{
    using namespace std::placeholders;
    
    // 在新线程中执行目标，避免阻塞executor
    std::thread{std::bind(&NavArmActionServer::execute, this, _1), goal_handle}.detach();
}

void NavArmActionServer::execute(const std::shared_ptr<GoalHandleNavArm> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "在独立线程中开始执行");
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<NavArmControl::Result>();
    auto feedback = std::make_shared<NavArmControl::Feedback>();
    
    // 设置重试参数
    int task_max_retries = 2;
    int step_max_retries = 5;
    
    try {
        // 调用编排器执行任务
        const TaskRunResult task_result = orchestrator_->run_task_with_retries(
            goal->waypoint_index,
            task_max_retries,
            step_max_retries,
            [goal_handle, feedback](double stage_value) {
                feedback->stage_feedback = stage_value;
                goal_handle->publish_feedback(feedback);
            });
        
        // 设置结果
            result->success = task_result.ok ? 1 : 0;
            result->failed_stage = task_result.failed_stage;
            result->task_attempt_used = task_result.task_attempt_used;
            result->message = task_result.message;

        // 检查是否被取消
        if (goal_handle->is_canceling()) {
            result->success = 0;
            goal_handle->canceled(result);
            RCLCPP_INFO(
                this->get_logger(),
                "目标已取消，waypoint_index=%ld",
                goal->waypoint_index);
            return;
        }

        if (task_result.ok) {
            goal_handle->succeed(result);
            RCLCPP_INFO(
                this->get_logger(),
                "目标执行成功，waypoint_index: %ld，任务尝试次数=%d",
                goal->waypoint_index,
                task_result.task_attempt_used);
        } else {
            goal_handle->abort(result);
            RCLCPP_WARN(
                this->get_logger(),
                "目标执行失败，waypoint_index: %ld，失败阶段=%d，信息=%s",
                goal->waypoint_index,
                task_result.failed_stage,
                task_result.message.c_str());
        }
        
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "执行过程中发生异常: %s", e.what());
        result->success = 0;
            result->failed_stage = 0;
            result->task_attempt_used = 0;
            result->message = std::string("执行异常: ") + e.what();
        goal_handle->abort(result);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavArmActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}