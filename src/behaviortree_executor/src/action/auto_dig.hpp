#include "rovr_interfaces/action/auto_dig.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using AutoDig = rovr_interfaces::action::AutoDig;
using namespace BT;

class AutoDigAction : public RosActionNode<AutoDig>
{
public:
    static BT::PortsList providedPorts()
{
    return {
        BT::InputPort<std::string>("action_name"),
    };
}
    AutoDigAction(const std::string &name, const BT::NodeConfig &conf,
                  const BT::RosNodeParams &params)
        : RosActionNode<AutoDig>(name, conf, params)
    {
    }

    bool setGoal(Goal &goal) override
    {
        // get inputs from the Input port
        bool backup_distance_sucess = getInput<double>("backup_distance", goal.backup_distance);
        // return true, if we were able to set the goal correctly.
        return backup_distance_success;;
    }

    NodeStatus onResultReceived(const WrappedResult &result) override
    {
        return NodeStatus::SUCCESS;
    }
};