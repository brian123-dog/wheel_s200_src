#include "multi_action_node/multi_action_node.hpp"
#include "multi_action_node/align_to_wall_action_bt_node.hpp"
#include "multi_action_node/nav_arm_action_bt_node.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace multi_action_node
{

MultiActionNode::MultiActionNode(
	const std::string & name,
	const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
	log_file_path_("/home/sunrise/wheel_S200_src/multi_action_node.log")
{
}

BT::NodeStatus MultiActionNode::tick()
{
	std::ofstream log_file(log_file_path_, std::ios::app);
	if (!log_file.is_open()) {
		RCLCPP_ERROR(
			rclcpp::get_logger("MultiActionNode"),
			"Failed to open log file: %s",
			log_file_path_.c_str());
		return BT::NodeStatus::FAILURE;
	}

	const auto now = std::chrono::system_clock::now();
	const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	std::tm tm_now {};
	localtime_r(&now_c, &tm_now);

	// 获取当前节点名称
	std::string node_name = this->name();
	std::string action_type;
	
	// 根据节点名称判断恢复行为类型
	if (node_name.find("Spin") != std::string::npos) {
		action_type = "旋转恢复";
	} else if (node_name.find("Wait") != std::string::npos) {
		action_type = "等待恢复";
	} else if (node_name.find("BackUp") != std::string::npos) {
		action_type = "后退恢复";
	} else if (node_name.find("Clear") != std::string::npos) {
		action_type = "清除代价地图恢复";
	} else {
		action_type = "未知恢复行为";
	}

	std::ostringstream line;
	line << std::put_time(&tm_now, "%F %T") 
	     << " [" << action_type << "] "
	     << "节点名称: " << node_name;
	log_file << line.str() << std::endl;

	RCLCPP_INFO(
		rclcpp::get_logger("MultiActionNode"),
		"[%s] 已记录到日志文件: %s",
		action_type.c_str(),
		log_file_path_.c_str());

	return BT::NodeStatus::SUCCESS;
}

}  // namespace multi_action_node

BT_REGISTER_NODES(factory)
{
	factory.registerNodeType<multi_action_node::MultiActionNode>("MultiActionNode");
	factory.registerNodeType<multi_action_node::CallAlignToWall>("CallAlignToWall");
	factory.registerNodeType<multi_action_node::CallNavArmAction>("CallNavArmAction");
}
