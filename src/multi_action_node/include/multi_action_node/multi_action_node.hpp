#ifndef MULTI_ACTION_NODE__MULTI_ACTION_NODE_HPP_
#define MULTI_ACTION_NODE__MULTI_ACTION_NODE_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"

namespace multi_action_node
{

class MultiActionNode : public BT::SyncActionNode
{
public:
    MultiActionNode(
        const std::string & name,
        const BT::NodeConfiguration & config);

    static BT::PortsList providedPorts()
    {
        // 如果需要从行为树传递参数，可以在这里定义端口
        // 例如：return { BT::InputPort<std::string>("log_message") };
        return {};
    }

    BT::NodeStatus tick() override;

private:
    std::string log_file_path_;
};

}  // namespace multi_action_node

#endif  // MULTI_ACTION_NODE__MULTI_ACTION_NODE_HPP_