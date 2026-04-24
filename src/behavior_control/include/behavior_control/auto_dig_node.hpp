#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rovr_interfaces/action/auto_dig.hpp"

class AutoDigAction : public BT::RosActionNode<rovr_interfaces::action::AutoDig> {
public:
    AutoDigAction(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) 
    : BT::RosActionNode<rovr_interfaces::action::AutoDig>(name, config, params) {}

    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("digging_start_position", "The position to lift the digging mechanism to"),
            InputPort<double>("digger_power", "The power to set the digger chain")
        };
    }
    
    bool setGoal(Goal& goal) override {
        double digging_start_position, digger_power;
        if (!getInput("digging_start_position", digging_start_position) || !getInput("digger_power", digger_power)) {
            auto node_ptr = node_.lock();
            RCLCPP_ERROR(node_ptr->get_logger(), "[%s]: Missing input for AutoDigAction! Ensure both [digging_start_position] and [digger_power] are provided.", name().c_str());
            return false;
        }

        goal.digging_start_position = digging_start_position;
        goal.digger_power = digger_power;

        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& result) override {
        auto node_ptr = node_.lock();
        if (node_ptr) {
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_WARN(node_ptr->get_logger(), "[%s]: Auto dig failed.", name().c_str());
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_ptr->get_logger(), "[%s]: Auto dig completed successfully.", name().c_str());
        }

        return BT::NodeStatus::SUCCESS;
    }
};