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
        BT::InputPort<double>("backup_distance")
    };
}
    AutoDigAction(const std::string &name, const BT::NodeConfig &conf,
                  const BT::RosNodeParams &params)
        : RosActionNode<AutoDig>(name, conf, params)
    {
    }

    bool setGoal(RosActionNode<AutoDig>::Goal &goal) override
    {
        // get inputs from the Input port
        auto backup_distance_success = getInput<double>("backup_distance", goal.backup_distance);
        // return true, if we were able to set the goal correctly.
        return (bool) backup_distance_success;
    }

    NodeStatus onResultReceived(const WrappedResult &result) override
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // The action server completed the dig successfully
                return NodeStatus::SUCCESS;
            case rclcpp_action::ResultCode::ABORTED:
                // Something went wrong (eg the dig got stuck or a sensor failed)
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