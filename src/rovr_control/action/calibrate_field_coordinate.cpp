#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using CalibrateFieldCoordinate = rovr_interfaces::action::CalibrateFieldCoordinates;

class CalibrateFieldCoordinateAction: public BT::RosActionNode<CalibrateFieldCoordinate> {
    public:
        CalibrateFieldCoordinateAction(
            const std::string& instance_name, 
            const BT::NodeConfig& conf)
        : BT::RosActionNode<CalibrateFieldCoordinate>(
            instance_name, 
            conf, 
            BT::RosNodeParams(std::make_shared<rclcpp::Node>("calibrate_field_coordinate_node"))) {}

        static BT::PortsList providedPorts() {
            return providedBasicPorts({});
        }

        BT::NodeStatus onResultReceived(const WrappedResult& result) override {
            //TODO: implement this :)
            return BT::NodeStatus::SUCCESS;
        }

        bool setGoal(Goal& goal) override {
            //TODO: implement this :)
            return true;
        }
};