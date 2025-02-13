#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

using CalibrateFieldCoordinate = rovr_interfaces::action::CalibrateFieldCoordinates;

class CalibrateFieldCoordinateAction : public BT::RosActionNode<CalibrateFieldCoordinate> {
    public:
        CalibrateFieldCoordinateAction(
            const std::string& name,
            const BT::NodeConfig& conf,
            const BT::RosNodeParams& params
        )
        : RosActionNode<CalibrateFieldCoordinate>(name, conf, params)
        {}

        BT::NodeStatus onResultReceived(const WrappedResult& wr) override {
            if (wr.result->success) {
                return BT::NodeStatus::SUCCESS;
            }
            
            return BT::NodeStatus::FAILURE;
        }
};

// Main method for the node
int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Spin the node
    // rclcpp::spin(std::make_shared<CalibrateFieldCoordinateAction>());

    // Free up any resources being used by the node
    rclcpp::shutdown();
    return 0;
}