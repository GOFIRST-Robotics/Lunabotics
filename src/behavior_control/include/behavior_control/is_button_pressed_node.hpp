#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

class IsButtonPressed : public BT::ConditionNode {
public:
    IsButtonPressed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::string>("joy_key"),
            BT::InputPort<int>("button_index") 
        };
    }

    BT::NodeStatus tick() override {
        std::string key;
        int index;
        if (!getInput("joy_key", key)) key = "joy_message";
        if (!getInput("button_index", index)) return BT::NodeStatus::FAILURE;

        auto joy_ptr = config().blackboard->getAnyPtr(key);
        if (!joy_ptr || joy_ptr->empty()) return BT::NodeStatus::FAILURE;

        const auto* joy_msg = joy_ptr->cast<sensor_msgs::msg::Joy>();
        
        if (index < 0 || index >= static_cast<int>(joy_msg->buttons.size())) return BT::NodeStatus::FAILURE;

        return (joy_msg->buttons[index] == 1) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};