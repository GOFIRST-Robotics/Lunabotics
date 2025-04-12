#include "rovr_interfaces/action/auto_dig.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

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

        bool setGoal(Goal& goal) override 
        {
            // get inputs from the Input port
            getInput("lift_digging_start_position", goal.lift_digging_start_position);
            getInput("lift_digging_end_position", goal.lift_digging_end_position);
            getInput("digger_belt_power", goal.digger_belt_power);
            // return true, if we were able to set the goal correctly.
            return true;
        }

        NodeStatus onResultReceived(__attribute__ ((unused)) const WrappedResult& result) override {
           return NodeStatus::SUCCESS;
        }
};