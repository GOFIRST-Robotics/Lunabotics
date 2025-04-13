#include "rclcpp/rclcpp.hpp"

#include "action/calibrate_field_coordinates.hpp"
#include "action/auto_dig.hpp"
#include "action/auto_offload.hpp"
#include "action/go_to_dig_location.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/tree_execution_server.hpp"

#include <thread>
#include <chrono>
#include <functional>

#include "std_srvs/srv/trigger.hpp"

class MyActionServer : public TreeExecutionServer { 
public: 
    MyActionServer(const rclcpp::NodeOptions& options) : TreeExecutionServer(options) { 
        // Register your custom nodes 
        factory().registerNodeType<AutoOffloadAction>("AutoOffload");
        factory().registerNodeType<GoToDigLocationAction>("GoToDigLocation");
        factory().registerNodeType<AutoDigAction>("AutoDig");
        factory().registerNodeType<CalibrateFieldCoordinateAction>("CalibrateFieldCoordinates");

    } 
private: 
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_; 
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_; 
}; 

int main(int argc, char** argv) { 
    rclcpp::init(argc, argv); 

    rclcpp::NodeOptions options;
    auto action_server = std::make_shared<MyActionServer>(options);
  
    // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
    // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                  std::chrono::milliseconds(250));
    exec.add_node(action_server->node());
    exec.spin();
    exec.remove_node(action_server->node());
    rclcpp::shutdown(); 
    return 0; 
}