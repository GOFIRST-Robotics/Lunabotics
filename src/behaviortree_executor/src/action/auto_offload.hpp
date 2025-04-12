//Someone please double check this is correct
#include "rovr_interfaces/action/auto_offload.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using AutoOffload = rovr_interfaces::action::AutoOffload;

class AutoOffloadAction : public BT::RosActionNode<AutoOffload>{
    public:
        AutoOffloadAction(
            const std::string& instance_name,
            const BT::NodeConfig& conf)
        
        : BT::RosActionNode<AutoOffload>(
            instance_name, 
            conf, 
            BT::RosNodeParams(std::make_shared<rclcpp::Node>("auto_offload_node"))){}

        static BT::PortsList providedPorts() {
            return providedBasicPorts({
                BT::InputPort<std::string>("action_name")
            });
        }

        //TODO: finish this
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
        
        //TODO: finish this
        bool setGoal(Goal& goal)override {
            return true;
        }     
};