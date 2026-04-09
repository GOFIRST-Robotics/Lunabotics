#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using CalibrateFieldCoordinates = rovr_interfaces::action::CalibrateFieldCoordinates;
using namespace BT;

class CalibrateFieldCoordinateAction : public RosActionNode<CalibrateFieldCoordinates>
{
public:
    static BT::PortsList providedPorts()
    {
        return
        {
            BT::InputPort<std::string>("action_name")
        };
    }
    CalibrateFieldCoordinateAction(const std::string &name, const BT::NodeConfig &conf,
                                   const BT::RosNodeParams &params)
        : RosActionNode<CalibrateFieldCoordinates>(name, conf, params)
    {
    }

    bool setGoal(dGoal &goal) override
    {
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult &result) override
    {
        return NodeStatus::SUCCESS;
    }
};