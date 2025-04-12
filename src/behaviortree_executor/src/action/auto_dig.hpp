#include "rovr_interfaces/action/auto_dig.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <cfloat>
using AutoDig = rovr_interfaces::action::AutoDig;
using namespace BT;

class AutoDigAction: public RosActionNode<AutoDig> {
    public:
        AutoDigAction(
            const std::string& instance_name, 
            const NodeConfig& conf)
        : RosActionNode<AutoDig>(
            instance_name, 
            conf, 
            RosNodeParams(std::make_shared<rclcpp::Node>("auto_dig_client"),"auto_dig")) {}

        static PortsList providedPorts() {
            return providedBasicPorts({
                InputPort<double>("lift_digging_start_position"),
                InputPort<double>("lift_digging_end_position"),
                InputPort<double>("digger_belt_power")
            });
        }

        bool setGoal(RosActionNode::Goal& goal) override 
        {
            // get inputs from the Input port
            getInput("lift_digging_start_position", goal.lift_digging_start_position);
            getInput("lift_digging_end_position", goal.lift_digging_end_position);
            getInput("digger_belt_power", goal.digger_belt_power);
            // return true, if we were able to set the goal correctly.
            return true;
        }

        NodeStatus onResultReceived(const WrappedResult& result) override {
           //Someone double check that this makes sense pls :)
           /* switch (result.code)
           {
               case rclcpp_action::ResultCode::SUCCEEDED:
                   RCLCPP_INFO(node_->get_logger(), "Calibration succeeded.");
                   return NodeStatus::SUCCESS;

               case rclcpp_action::ResultCode::ABORTED:
                   RCLCPP_ERROR(node_->get_logger(), "Calibration aborted.");
                   return NodeStatus::FAILURE;

               case rclcpp_action::ResultCode::CANCELED:
                   RCLCPP_WARN(node_->get_logger(), "Calibration canceled.");
                   return NodeStatus::FAILURE;

               default:
                   RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
                   return NodeStatus::FAILURE;
           } */
           return NodeStatus::SUCCESS;
        }
};