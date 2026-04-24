#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"

class CalibrateFieldCoordinatesAction : public BT::RosActionNode<rovr_interfaces::action::CalibrateFieldCoordinates> {
public:
    CalibrateFieldCoordinatesAction(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) 
    : BT::RosActionNode<rovr_interfaces::action::CalibrateFieldCoordinates>(name, config, params) {}

    static BT::PortsList providedPorts() {
        return {};
    }
    
    bool setGoal(Goal& ) override {
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& result) override {
        auto node_ptr = node_.lock();
        if (node_ptr) {
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_WARN(node_ptr->get_logger(), "[%s]: Calibrate field coordinates failed.", name().c_str());
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_ptr->get_logger(), "[%s]: Calibrate field coordinates completed successfully.", name().c_str());
        }

        return BT::NodeStatus::SUCCESS;
    }
};