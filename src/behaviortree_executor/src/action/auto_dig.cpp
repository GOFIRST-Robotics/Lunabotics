#include "rovr_interfaces/action/auto_dig.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using AutoDig = rovr_interfaces::action::AutoDig;

class AutoDigAction: public BT::RosActionNode<AutoDig> {
    public:
        AutoDigAction(
            const std::string& instance_name, 
            const BT::NodeConfig& conf)
        : BT::RosActionNode<AutoDig>(
            instance_name, 
            conf, 
            BT::RosNodeParams(std::make_shared<rclcpp::Node>("auto_dig_node"))) {}

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