#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using CalibrateFieldCoordinates = rovr_interfaces::action::CalibrateFieldCoordinates;
using namespace BT;

class CalibrateFieldCoordinateAction : public RosActionNode<CalibrateFieldCoordinates>
{
public:
    CalibrateFieldCoordinateAction(const std::string &name, const BT::NodeConfig &conf,
                                   const BT::RosNodeParams &params)
        : RosActionNode<CalibrateFieldCoordinates>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    bool setGoal(__attribute__((unused)) Goal &goal) override
    {
        return true;
    }

    NodeStatus onResultReceived(__attribute__((unused)) const WrappedResult &result) override
    {
        return NodeStatus::SUCCESS;
    }
};