#include"rovr_interfaces/action/auto_dig_nav_offload.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
// actual file is in behaviortree_ros2 folder
using namespace BT;

class AutoDigNavOffLoadAction : public RosAction<AutoDig>
{
    public:
    static BT::PortsList providedPorts()
    {
        return
        {
          BT::InputPort<std::string>("action_name")  
        };
    }

    AutoDigNavOffLoadAction(const std::string &name, const BT::NodeConfiguration &config,
    const BT::RosNodeParams &params) : RosActionNode<AutoDig>(name, config, params)
    {
    }

    bool setGoal(Goal &goal) override
    {
        bool target_x_success = getInput<double>("target_x", goal.target_x);
        bool target_y_success = getInput<double>("target_y", goal.target_y);
        return target_x_success && target_y_success;
    }

    NodeStatus onResultReceived(const WrappedResult & result) override
    {
        return NodeStatus::SUCCESS;
    }
}