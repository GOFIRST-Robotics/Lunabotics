// Someone please double check this is correct
#include "rovr_interfaces/action/auto_offload.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using AutoOffload = rovr_interfaces::action::AutoOffload;
using namespace BT;

class AutoOffloadAction : public RosActionNode<AutoOffload>
{
public:
    AutoOffloadAction(const std::string &name, const BT::NodeConfig &conf,
                      const BT::RosNodeParams &params)

        : RosActionNode<AutoOffload>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<double>("lift_dumping_position"),
            InputPort<double>("digger_belt_power"),
        });
    }

    bool setGoal(Goal &goal) override
    {
        // get inputs from the Input port
        getInput("lift_dumping_position", goal.lift_dumping_position);
        getInput("digger_belt_power", goal.digger_belt_power);
        // return true, if we were able to set the goal correctly.
        return true;
    }

    // TODO: finish this
    NodeStatus onResultReceived(__attribute__((unused)) const WrappedResult &result) override
    {
        return NodeStatus::SUCCESS;
    }
};