#include <chrono>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rovr_interfaces/srv/addition.hpp"
#include "interface.h"

struct ROS2Bridge {
    rclcpp::Node::SharedPtr node;
};

extern "C" {

ROS2ContextHandle init_ros2(const char* node_name) {
    if (!rclcpp::ok()) {
        // Pass dummy args for initialization
        int argc = 0;
        char** argv = nullptr;
        rclcpp::init(argc, argv);
    }
    
    auto bridge = new ROS2Bridge();
    bridge->node = rclcpp::Node::make_shared(node_name);
    return static_cast<ROS2ContextHandle>(bridge);
}

void shutdown_ros2(ROS2ContextHandle handle) {
    if (!handle) return;
    auto bridge = static_cast<ROS2Bridge*>(handle);
    
    delete bridge;
    
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

int call_add_two_ints_service(ROS2ContextHandle handle, const char* service_name, int a, int b, int* out_sum) {
    if (!handle || !out_sum) return -1;
    auto bridge = static_cast<ROS2Bridge*>(handle);

    auto client = bridge->node->create_client<rovr_interfaces::srv::Addition>(service_name);

    if (!client->wait_for_service(std::chrono::seconds(2))) {
        return -1; 
    }

    auto request = std::make_shared<rovr_interfaces::srv::Addition::Request>();
    request->a = a;
    request->b = b;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(bridge->node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        *out_sum = result_future.get()->sum;
        return 0;
    }

    return -1;
}

} // extern "C"

