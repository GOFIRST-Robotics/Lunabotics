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
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y")
        };
    }
    GoToDigLocationAction(const std::string& name, const BT::NodeConfig& conf,
        const BT::RosNodeParams& params)
        : RosActionNode<GoToDigLocation>(name, conf, params)
    {
    }

    bool setGoal(__attribute__((unused)) Goal &goal) override
    {
        goal.target_x = getInput<double>("target_x", goal.target_x);
        goal.target_y = getInput<double>("target_y", goal.target_y);
        return true;
    }

    NodeStatus onResultReceived(__attribute__((unused)) const WrappedResult &result) override
    {
        return NodeStatus::SUCCESS;
    }
};
