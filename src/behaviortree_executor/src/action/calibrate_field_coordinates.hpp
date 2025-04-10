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
            return providedBasicPorts({
                BT::InputPort<std::string>("action_name")
            });
        }

        BT::NodeStatus onResultReceived(const WrappedResult& result) override {
            //Someone double check that this makes sense pls :)
            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(node_->get_logger(), "Calibration succeeded.");
                    return BT::NodeStatus::SUCCESS;

                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(node_->get_logger(), "Calibration aborted.");
                    return BT::NodeStatus::FAILURE;

                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(node_->get_logger(), "Calibration canceled.");
                    return BT::NodeStatus::FAILURE;

                default:
                    RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
                    return BT::NodeStatus::FAILURE;
            }
        }

        bool setGoal(Goal& goal) override {
            //TODO: implement this :)
            return true;
        }
};