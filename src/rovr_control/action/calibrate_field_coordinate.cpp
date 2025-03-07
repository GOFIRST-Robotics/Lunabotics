#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using CalibrateFieldCoordinate = rovr_interfaces::action::CalibrateFieldCoordinates;

class CalibrateFieldCoordinateAction: public BT::RosActionNode<CalibrateFieldCoordinate> {
    public:
        CalibrateFieldCoordinateAction(
            const std::string& instance_name, 
            const BT::NodeConfig& conf,
            const BT::RosNodeParams& params)
        : BT::RosActionNode<CalibrateFieldCoordinate>(instance_name, conf, params) {}

        BT::NodeStatus onResultReceived(const WrappedResult& result) override {
            //TODO: implement this :)
            return BT::NodeStatus::SUCCESS;
        }

        bool setGoal(Goal& goal) override {
            //TODO: implement this :)
            return true;
        }
};

int main(int argc, char** argv) { 
    rclcpp::init(argc, argv); 

    // Create a ROS node
    auto node = std::make_shared<rclcpp::Node>("calibrate_field_coordinate_node");

    BT::NodeConfig config;
    BT::RosNodeParams ros_node_params;
    ros_node_params.nh = node;

    // Create an instance of your action class, passing the ROS node
    auto action_node = std::make_shared<CalibrateFieldCoordinateAction>("calibrate_field_coordinate_action", config, ros_node_params);

    rclcpp::spin(node);
    rclcpp::shutdown(); 
    return 0;
}