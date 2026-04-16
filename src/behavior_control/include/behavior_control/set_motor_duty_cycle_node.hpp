#include "behaviortree_ros2/bt_service_node.hpp"
#include "rovr_interfaces/srv/motor_command_set.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace BT;

class SetMotorDutyCycle : public RosServiceNode<rovr_interfaces::srv::MotorCommandSet> {
public:
    SetMotorDutyCycle(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : RosServiceNode<rovr_interfaces::srv::MotorCommandSet>(name, conf, params) {}

    static PortsList providedPorts() {
        return {
            InputPort<int>("can_id"),
            InputPort<float>("duty_cycle"),
            InputPort<float>("power_limit", 0.5f)
        };
    }

    bool setServiceRequest(std::shared_ptr<Request>& request) override {
        int can_id;
        float duty_cycle, power_limit;

        if (!getInput("can_id", can_id) || !getInput("duty_cycle", duty_cycle)) return false;
        getInput("power_limit", power_limit);

        request->can_id = can_id;
        request->type = "duty_cycle";
        request->value = duty_cycle;
        request->power_limit = power_limit;
        return true;
    }

    NodeStatus onResponseReceived(const Response& response) override {
        return response.success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};