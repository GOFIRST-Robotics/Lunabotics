#include "rovr_interfaces/action/go_to_dig_location.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using GoToDigLocation = rovr_interfaces::action::GoToDigLocation;
using namespace BT;

class GoToDigLocationAction : public RosActionNode<GoToDigLocation>
{
public:
    static BT::PortsList providedPorts()
    {
        return
        {
            BT::InputPort<std::string>("action_name"),
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y")
        };
    }
    GoToDigLocationAction(const std::string& name, const BT::NodeConfig& conf,
        const BT::RosNodeParams& params)
        : RosActionNode<GoToDigLocation>(name, conf, params)
    {
    }

    bool setGoal(RosActionNode<GoToDigLocation>::Goal &goal) override
    {
        auto x_success = getInput<double>("x", goal.x);
        auto y_success = getInput<double>("y", goal.y);
        return (bool) (x_success && y_success);
    }

    NodeStatus onResultReceived(const WrappedResult &result) override
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // The action server completed the go to dig location successfully
                return NodeStatus::SUCCESS;
            case rclcpp_action::ResultCode::ABORTED:
                // Something went wrong (eg the robot got stuck or a sensor failed)
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
