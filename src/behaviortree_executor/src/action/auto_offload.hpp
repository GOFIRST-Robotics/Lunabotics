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
static BT::PortsList providedPorts()
{
    return {
        BT::InputPort<double>("lift_dumping_position"),
        BT::InputPort<double>("digger_chain_power") // Updated name
    };
}
    AutoOffloadAction(const std::string &name, const BT::NodeConfig &conf,
                      const BT::RosNodeParams &params)

        : RosActionNode<AutoOffload>(name, conf, params)
    {
    }

    bool setGoal(Goal &goal) override
    {
        // get inputs from the Input port
        getInput("lift_dumping_position", goal.lift_dumping_position);
        getInput("digger_belt_power", goal.digger_belt_power);
        // return true, if we were able to set the goal correctly.
        return true;
    }

    // Addeed a switch statement to handle different result codes from the action server
    NodeStatus onResultReceived(__attribute__((unused)) const WrappedResult &result) override
    {
        NodeStatus onResultReceived(const WrappedResult &result) override
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // The action server completed the offload successfully
                return NodeStatus::SUCCESS;

            case rclcpp_action::ResultCode::ABORTED:
                // Something went wrong (eg the lift jammed or a sensor failed)
                RCLCPP_ERROR(node_->get_logger(), "AutoOffload aborted!");
                return NodeStatus::FAILURE;

            case rclcpp_action::ResultCode::CANCELED:
                // The action was canceled
                RCLCPP_WARN(node_->get_logger(), "AutoOffload canceled.");
                return NodeStatus::FAILURE;

            default:
                // Any other weirdness should generally be a failure
                return NodeStatus::FAILURE;
        }
    }
    }
};