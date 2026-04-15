#include <iostream>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

class LogString : public BT::SyncActionNode {
public:
    LogString(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Logger node_logger)
    : BT::SyncActionNode(name, config), logger(node_logger) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("message") };
    }

    BT::NodeStatus tick() override {
        auto msg = getInput<std::string>("message");
        
        if (!msg) {
            RCLCPP_ERROR(logger, "LogString: Port [message] is empty or not mapped! Error: %s", msg.error().c_str());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(logger, "LogString: %s", msg.value().c_str());
        
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Logger logger;
};