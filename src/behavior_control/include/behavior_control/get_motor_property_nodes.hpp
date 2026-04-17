#include "behaviortree_ros2/bt_service_node.hpp"
#include "rovr_interfaces/srv/motor_command_get.hpp"

using namespace BT;

template <typename T>
class GetMotorBase : public RosServiceNode<rovr_interfaces::srv::MotorCommandGet> {
public:
    GetMotorBase(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params, std::string type)
        : RosServiceNode<rovr_interfaces::srv::MotorCommandGet>(name, conf, params), type_(type) {}

    static PortsList providedPorts() {
        return {
            InputPort<uint32_t>("can_id", "The CAN ID of the VESC"),
            OutputPort<T>("value_out", "The data returned by the motor")
        };
    }

    bool setRequest(typename Request::SharedPtr& request) override {
        uint32_t can_id;
        if (!getInput("can_id", can_id)) return false;

        request->can_id = can_id;
        request->type = type_;
        return true;
    }

    NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override {
        if (response->success) {
            setOutput("value_out", static_cast<T>(response->data));
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
    std::string type_;
};

class GetMotorCurrent : public GetMotorBase<double> {
public:
    GetMotorCurrent(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : GetMotorBase(name, conf, params, "current") {}
};

class GetMotorPosition : public GetMotorBase<double> {
public:
    GetMotorPosition(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : GetMotorBase(name, conf, params, "position") {}
};

class GetMotorVelocity : public GetMotorBase<double> {
public:
    GetMotorVelocity(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : GetMotorBase(name, conf, params, "velocity") {}
};

class GetMotorDutyCycle : public GetMotorBase<double> {
public:
    GetMotorDutyCycle(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : GetMotorBase(name, conf, params, "duty_cycle") {}
};