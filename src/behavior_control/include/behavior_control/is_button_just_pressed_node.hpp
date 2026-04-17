#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_cpp/condition_node.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

class IsButtonJustPressed : public BT::ConditionNode {
public:
    IsButtonJustPressed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), last_state_(false) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<int>("button_index"),
            BT::InputPort<sensor_msgs::msg::Joy>("input_source")
        };
    }
    
    BT::NodeStatus tick() override {
        int index;
        sensor_msgs::msg::Joy joy_msg;

        if (!getInput("button_index", index) || !getInput("input_source", joy_msg)) {
            return BT::NodeStatus::FAILURE;
        }

        // Bounds check
        if (index < 0 || index >= static_cast<int>(joy_msg.buttons.size())) {
            return BT::NodeStatus::FAILURE;
        }

        bool current_state = (joy_msg.buttons[index] == 1);
        BT::NodeStatus status = BT::NodeStatus::FAILURE;

        // Logic: SUCCESS only if it was false and is now true (Rising Edge)
        if (current_state && !last_state_) {
            status = BT::NodeStatus::SUCCESS;
        }

        // Update state for the next tick
        last_state_ = current_state;
        
        return status;
    }

private:
    bool last_state_; 
};