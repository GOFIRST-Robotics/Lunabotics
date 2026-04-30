#include "behaviortree_ros2/bt_service_node.hpp"
#include "rovr_interfaces/srv/motor_command_set.hpp"

using namespace BT;

template <typename T>
class SetMotorBase : public RosServiceNode<rovr_interfaces::srv::MotorCommandSet> {
public:
    SetMotorBase(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params, std::string type)
        : RosServiceNode<rovr_interfaces::srv::MotorCommandSet>(name, conf, params), type_(type) {}

    static PortsList providedPorts() {
        return {
            InputPort<uint32_t>("can_id", "The CAN ID of the VESC"),
            InputPort<T>("value_in", "The data to send to the motor")
        };
    }

    bool setRequest(typename Request::SharedPtr& request) override {
        uint32_t can_id;
        T value_in;

        if (!getInput("can_id", can_id) || !getInput("value_in", value_in)) return false;

        request->can_id = can_id;
        request->type = type_;
        request->value = static_cast<float(value_in);
        return true;
    }

    NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override {
        return response->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

    void halt() override {
        uint32_t can_id;
        // We retrieve the can_id from the ports to know which motor to stop
        if (this->getInput("can_id", can_id)) {
            auto stop_request = std::make_shared<rovr_interfaces::srv::MotorCommandSet::Request>();
            stop_request->can_id = can_id;
            stop_request->type = "duty_cycle"; // We choose duty_cycle as a safe way to stop the motor, NOT POSITION or VELOCITY which might have unintended consequences.
            stop_request->value = 0.0f;
            
            RCLCPP_INFO(this->logger(), "Halt called: Sending stop command to motor %d", can_id);
            
            // srv_instance_ is protected in the header you shared, 
            // so we can access the client directly for a quick async call.
            this->srv_instance_->service_client->async_send_request(stop_request);
        }
        
        // CRITICAL: Call the base class halt() so the BT node state (RUNNING -> IDLE) 
        // is updated correctly!
        RosServiceNode<rovr_interfaces::srv::MotorCommandSet>::halt();
    }
    
private:
    std::string type_;
};

class SetMotorPosition : public SetMotorBase<double> {
public:
    SetMotorPosition(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : SetMotorBase(name, conf, params, "position") {}
};

class SetMotorVelocity : public SetMotorBase<double> {
public:
    SetMotorVelocity(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : SetMotorBase(name, conf, params, "velocity") {}
};

class SetMotorDutyCycle : public SetMotorBase<double> {
public:
    SetMotorDutyCycle(const std::string& name, const NodeConfiguration& conf, const RosNodeParams& params)
        : SetMotorBase(name, conf, params, "duty_cycle") {}
};