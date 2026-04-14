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
        BT::InputPort<std::string>("action_name")
    };
}
    AutoOffloadAction(const std::string &name, const BT::NodeConfig &conf,
                      const BT::RosNodeParams &params)

        : RosActionNode<AutoOffload>(name, conf, params)
    {
    }

    bool setGoal(Goal &goal) override
    {
        bool lift_dumping_position_succss = getInput<double>("lift_dumping_position",
                                                     goal.lift_dumping_position);
                                                     
        // return true, if we were able to set the goal correctly.
        return lift_dumping_position_success;

    // Added a switch statement to handle different result codes from the action server
    NodeStatus onResultReceived(const WrappedResult &result) override
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // The action server completed the offload successfully
                return NodeStatus::SUCCESS;

            case rclcpp_action::ResultCode::ABORTED:
                // Something went wrong (eg the lift jammed or a sensor failed)
                return NodeStatus::FAILURE;

            case rclcpp_action::ResultCode::CANCELED:
                // The action was canceled
                return NodeStatus::FAILURE;

            default:
                // Any other weirdness should generally be a failure
                return NodeStatus::FAILURE;
        }
    }
};