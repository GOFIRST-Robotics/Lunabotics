#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "rclcpp/rclcpp.hpp"

class IsButtonPressed : public BT::ConditionNode {
public:
    IsButtonPressed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<int>("button_index")
            BT::InputPort<sensor_msgs::msg::Joy>("input_source")
        };
    }

    BT::NodeStatus tick() override {
        int index;
        sensor_msgs::msg::Joy joy_msg;

        if (!getInput("button_index", index) || !getInput("input_source", joy_msg)) {
            return BT::NodeStatus::FAILURE;
        }

        if (index < 0 || index >= static_cast<int>(joy_msg.buttons.size())) {
            auto node_ptr = node_.lock();
            RCLCPP_WARN(node_ptr->get_logger(), "[%s]: Button index out of range.", name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (joy_msg.buttons[index] == 1) {
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::FAILURE;
    }
};