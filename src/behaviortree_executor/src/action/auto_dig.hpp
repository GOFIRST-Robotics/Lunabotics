#include "rovr_interfaces/action/auto_dig.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using AutoDig = rovr_interfaces::action::AutoDig;
using namespace BT;

class AutoDigAction : public RosActionNode<AutoDig>
{
public:
    AutoDigAction(const std::string &name, const BT::NodeConfig &conf,
                  const BT::RosNodeParams &params)
        : RosActionNode<AutoDig>(name, conf, params) {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({InputPort<double>("lift_digging_start_position"),
                                   InputPort<double>("lift_digging_end_position"),
                                   InputPort<double>("digger_belt_power")});
    }

    bool setGoal(Goal &goal) override
    {
        // get inputs from the Input port
        getInput("lift_digging_start_position", goal.lift_digging_start_position);
        getInput("lift_digging_end_position", goal.lift_digging_end_position);
        getInput("digger_belt_power", goal.digger_belt_power);
        // return true, if we were able to set the goal correctly.
        return true;
    }

    NodeStatus onResultReceived(__attribute__((unused)) const WrappedResult &result) override
    {
        return NodeStatus::SUCCESS;
    }
};