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
            return providedBasicPorts({});
        }

        //TODO: finish this
        BT::NodeStatus onResultReceived(const WrappedResult& result) override {
            return BT::NodeStatus::SUCCESS;
        }
        
        //TODO: finish this
        bool setGoal(Goal& goal)override {
            return true;
        }     
};