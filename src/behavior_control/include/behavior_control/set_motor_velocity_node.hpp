#include "behaviortree_ros2/bt_service_node.hpp"
#include "rovr_interfaces/srv/motor_command_set.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace BT;

class SetMotorVelocity : public RosServiceNode<rovr_interfaces::srv::MotorCommandSet> {
public:
    SetMotorVelocity(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : RosServiceNode<rovr_interfaces::srv::MotorCommandSet>(name, conf, params) {}

    // Define the ports required by this node
    static PortsList providedPorts() {
        return {
            InputPort<int>("can_id", "The CAN ID of the motor"),
            InputPort<double>("velocity", "Target velocity in RPM"),
            InputPort<double>("power_limit", 1.0, "Power limit (0.0 to 1.0)")
        };
    }

    // This method is called when the node is ticked
    bool setServiceRequest(std::shared_ptr<Request>& request) override {
        int can_id;
        double velocity, power_limit;

        if (!getInput("can_id", can_id) || !getInput("velocity", velocity)) {
            return false;
        }
        getInput("power_limit", power_limit);

        request->can_id = can_id;
        request->type = "velocity";
        request->value = velocity;
        request->value2 = power_limit; // Using value2 as power_limit per your interface
        return true;
    }

    // Called when the service returns a response
    NodeStatus onResponseReceived(const Response& response) override {
        if (response.success) {
            return NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(node_.lock()->get_logger(), "SetMotorVelocity failed for ID %d", request_->can_id);
        return NodeStatus::FAILURE;
    }
};