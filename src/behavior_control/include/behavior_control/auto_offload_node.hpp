#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rovr_interfaces/action/auto_offload.hpp"

class AutoOffloadAction : public BT::RosActionNode<rovr_interfaces::action::AutoOffload> {
public:
    AutoOffloadAction(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) 
    : BT::RosActionNode<rovr_interfaces::action::AutoOffload>(name, config, params) {}

    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("dumping_position", "The position to lift the dumping mechanism to"),
            InputPort<double>("digger_power", "The power to set the digger chain")
        };
    }
    
    bool setGoal(Goal& goal) override {
        double dumping_position, digger_power;
        if (!getInput("dumping_position", dumping_position) || !getInput("digger_power", digger_power)) {
            auto node_ptr = node_.lock();
            RCLCPP_ERROR(node_ptr->get_logger(), "[%s]: Missing input for AutoOffloadAction! Ensure both [dumping_position] and [digger_power] are provided.", name().c_str());
            return false;
        }

        goal.dumping_position = dumping_position;
        goal.digger_power = digger_power;

        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& result) override {
        auto node_ptr = node_.lock();
        if (node_ptr) {
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_WARN(node_ptr->get_logger(), "[%s]: Auto offload failed.", name().c_str());
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_ptr->get_logger(), "[%s]: Auto offload completed successfully.", name().c_str());
        }

        return BT::NodeStatus::SUCCESS;
    }
};