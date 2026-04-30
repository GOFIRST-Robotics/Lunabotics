#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

class GetUserInput : public BT::SyncActionNode {
public:
    GetUserInput(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<int>("button_index"),
            BT::InputPort<sensor_msgs::msg::Joy>("input_source"),
            BT::InputPort<int>("input_type", "0 for button, 1 for axis"),
            BT::OutputPort<float>("output")
        };
    }

    BT::NodeStatus tick() override {
        int index;
        sensor_msgs::msg::Joy joy_msg;
        int input_type;

        if (!getInput("button_index", index) || !getInput("input_source", joy_msg) || !getInput("input_type", input_type)) {
            return BT::NodeStatus::FAILURE;
        }

        if (index >= 0 && input_type == 0) {
            if (index >= static_cast<int>(joy_msg.buttons.size())) {
                return BT::NodeStatus::FAILURE;
            }

            setOutput("output", static_cast<float>(joy_msg.buttons[index]));

            return BT::NodeStatus::SUCCESS;
        } else if (input_type == 1) {
            if (index >= static_cast<int>(joy_msg.axes.size())) {
                return BT::NodeStatus::FAILURE;
            }
            
            setOutput("output", static_cast<float>(joy_msg.axes[index]));

            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::FAILURE;
    }
};