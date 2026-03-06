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
        goal.backup_distance = getInput<double>("backup_distance", goal.backup_distance);
        goal.digger_chain_power = getInput<double>("digger_chain_power", goal.digger_chain_power);
        // return true, if we were able to set the goal correctly.
        return true;
    }

    NodeStatus onResultReceived(__attribute__((unused)) const WrappedResult &result) override
    {
        return NodeStatus::SUCCESS;
    }
};