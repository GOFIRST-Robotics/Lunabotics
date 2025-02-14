#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

using CalibrateFieldCoordinate = rovr_interfaces::action::CalibrateFieldCoordinates;

class CalibrateFieldCoordinateAction: public BT::RosActionNode<CalibrateFieldCoordinate> {
public:
    CalibrateFieldCoordinateAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosActionNode<CalibrateFieldCoordinate>(name, conf, params) {}

    // This is called when the TreeNode is ticked and it should
    // send the request to the action server
    bool setGoal(BT::RosActionNode<CalibrateFieldCoordinate>::Goal& goal) override {
        return true;
    }
    
    // Callback executed when the reply is received.
    // Based on the reply you may decide to return SUCCESS or FAILURE.
    BT::NodeStatus onResultReceived(const WrappedResult& wr) override {
        return (wr.result->success) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// Main method for the node
int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    auto node = std::make_shared<rclcpp::Node>("calibrate_field_coordinates_action_client");
    
    // provide the ROS node and the name of the action service
    BT::RosNodeParams params; 
    params.nh = node;
    params.default_port_value = "calibrate_field_coordinates";
    factory.registerNodeType<CalibrateFieldCoordinateAction>("CalibrateFieldCoordinate", params);

    // Free up any resources being used by the node
    rclcpp::shutdown();
    return 0;
}