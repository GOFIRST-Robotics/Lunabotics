#include "behaviortree_cpp/condition_node.h"
#include "sensor_msgs/msg/joy.hpp"

class IsButtonJustPressed : public BT::ConditionNode {
public:
    IsButtonJustPressed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), last_state_(false) {}

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

        auto cur_ptr = config().blackboard->getAnyPtr(key);
        if (!cur_ptr || cur_ptr->empty()) return BT::NodeStatus::FAILURE;
        const auto* cur_joy = cur_ptr->cast<sensor_msgs::msg::Joy>();

        if (index < 0 || index >= static_cast<int>(cur_joy->buttons.size())) {
            return BT::NodeStatus::FAILURE;
        }

        bool currently_pressed = (cur_joy->buttons[index] == 1);
        BT::NodeStatus status = BT::NodeStatus::FAILURE;

        if (currently_pressed && !last_state_) {
            status = BT::NodeStatus::SUCCESS;
        }

        last_state_ = currently_pressed;
        
        return status;
    }

private:
    bool last_state_;
};