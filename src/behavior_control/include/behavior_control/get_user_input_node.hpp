#include "behaviortree_cpp/action_node.h"
#include "sensor_msgs/msg/joy.hpp"

class GetUserInput : public BT::SyncActionNode {
public:
    GetUserInput(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::string>("joy_key"),
            BT::InputPort<int>("index"),
            BT::InputPort<int>("input_type", "0 for button, 1 for axis"),
            BT::OutputPort<float>("output")
        };
    }

    BT::NodeStatus tick() override {
        std::string key;
        if (!getInput("joy_key", key)) key = "joy_message"; // Default

        auto joy_ptr = config().blackboard->getAnyPtr(key);
        if (!joy_ptr || joy_ptr->empty()) return BT::NodeStatus::FAILURE;

        const auto* joy_msg = joy_ptr->cast<sensor_msgs::msg::Joy>();

        int index, input_type;
        if (!getInput("index", index) || !getInput("input_type", input_type)) return BT::NodeStatus::FAILURE;

        if (input_type == 0) {
            if (index < 0 || index >= static_cast<int>(joy_msg->buttons.size())) return BT::NodeStatus::FAILURE;
            setOutput("output", static_cast<float>(joy_msg->buttons[index]));
        } 
        else if (input_type == 1) { // Axis Path
            if (index < 0 || index >= static_cast<int>(joy_msg->axes.size())) return BT::NodeStatus::FAILURE;
            setOutput("output", joy_msg->axes[index]);
        }

        return BT::NodeStatus::SUCCESS;
    }
};