#include "behaviortree_ros2/bt_service_node.hpp"
#include "rovr_interfaces/srv/motor_command_set.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace BT;

class SetMotorVelocity : public RosServiceNode<rovr_interfaces::srv::MotorCommandSet> {
public:
    SetMotorVelocity(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : RosServiceNode<rovr_interfaces::srv::MotorCommandSet>(name, conf, params) {}

    static PortsList providedPorts() {
        return {
            InputPort<int>("can_id"),
            InputPort<float>("velocity"),
            InputPort<float>("power_limit", 0.5f, "Max power (0.0 to 1.0)")
        };
    }

    bool setServiceRequest(std::shared_ptr<Request>& request) override {
        int can_id;
        float velocity, power_limit;

        if (!getInput("can_id", can_id) || !getInput("velocity", velocity)) return false;
        getInput("power_limit", power_limit);

        request->can_id = can_id;
        request->type = "velocity";
        request->value = velocity;
        request->power_limit = power_limit; // Now using the explicit field
        return true;
    }

    NodeStatus onResponseReceived(const Response& response) override {
        return response.success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};