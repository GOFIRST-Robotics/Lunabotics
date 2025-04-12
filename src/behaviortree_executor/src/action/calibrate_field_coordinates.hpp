#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using CalibrateFieldCoordinates = rovr_interfaces::action::CalibrateFieldCoordinates;
using namespace BT;

class CalibrateFieldCoordinateAction : public RosActionNode<CalibrateFieldCoordinates>
{
public:
    CalibrateFieldCoordinateAction(
        const std::string& instance_name,
        const NodeConfig& conf)
        : RosActionNode<CalibrateFieldCoordinates>(
              instance_name,
              conf,
              RosNodeParams(std::make_shared<rclcpp::Node>("calibrate_field_coordinates_client", "calibrate_field_coordinates"))) 
    {
    }

    bool setGoal(__attribute__ ((unused)) Goal &goal) override
    {
        return true;
    }

    NodeStatus onResultReceived(__attribute__ ((unused)) const WrappedResult& result) override
    {
        return NodeStatus::SUCCESS;
    }
};