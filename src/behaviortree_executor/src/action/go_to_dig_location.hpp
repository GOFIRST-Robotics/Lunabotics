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

    bool setGoal(Goal &goal) override
    {
        bool x_success = getInput<double>("x", goal.x);
        bool y_success = getInput<double>("y", goal.y);
        return x_success && y_success;
    }

    NodeStatus onResultReceived(const WrappedResult &result) override
    {
        return NodeStatus::SUCCESS;
    }
};
