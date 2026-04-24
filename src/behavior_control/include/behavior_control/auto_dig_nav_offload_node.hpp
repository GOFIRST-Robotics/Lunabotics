#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rovr_interfaces/action/auto_dig_nav_offload.hpp"

class AutoDigNavOffloadAction : public BT::RosActionNode<rovr_interfaces::action::AutoDigNavOffload> {
public:
    AutoDigNavOffloadAction(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) 
    : BT::RosActionNode<rovr_interfaces::action::AutoDigNavOffload>(name, config, params) {}

    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("digging_start_position", "The position to lift the digging mechanism to"),
            InputPort<double>("digger_power", "The power to set the digger chain"),
            InputPort<double>("backward_distance", "The distance to move backwards after digging"),
            InputPort<double>("x_pos", "The x position to navigate to after digging"),
            InputPort<double>("y_pos", "The y position to navigate to after digging")
        };
    }
    
    bool setGoal(Goal& goal) override {
        double digging_start_position, digger_power, backward_distance, x_pos, y_pos;
        if (!getInput("digging_start_position", digging_start_position) 
        || !getInput("digger_power", digger_power)
        || !getInput("backward_distance", backward_distance)
        || !getInput("x_pos", x_pos)
        || !getInput("y_pos", y_pos)) {
            auto node_ptr = node_.lock();
            RCLCPP_ERROR(node_ptr->get_logger(), "[%s]: Missing input for AutoDigAction! Ensure all inputs are provided.", name().c_str());
            return false;
        }

        goal.digging_start_position = digging_start_position;
        goal.digger_power = digger_power;
        goal.backward_distance = backward_distance;
        goal.x_pos = x_pos;
        goal.y_pos = y_pos;

        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& result) override {
        auto node_ptr = node_.lock();
        if (node_ptr) {
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_WARN(node_ptr->get_logger(), "[%s]: Auto dig nav offload failed.", name().c_str());
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_ptr->get_logger(), "[%s]: Auto dig nav offload completed successfully.", name().c_str());
        }

        return BT::NodeStatus::SUCCESS;
    }
};