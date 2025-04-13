#include "rclcpp/rclcpp.hpp"

#include "action/calibrate_field_coordinates.hpp"
#include "action/auto_dig.hpp"
#include "action/auto_offload.hpp"
#include "action/go_to_dig_location.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include <thread>
#include <chrono>
#include <functional>

#include "std_srvs/srv/trigger.hpp"

class BehaviorTreeExecutor
{
public:
    BehaviorTreeExecutor() : node(std::make_shared<rclcpp::Node>("bt_executor"))
    {
        // Register your custom nodes

        factory.registerNodeType<AutoOffloadAction>("AutoOffload", node);
        factory.registerNodeType<GoToDigLocationAction>("GoToDigLocation", node);
        factory.registerNodeType<AutoDigAction>("AutoDig", node);
        factory.registerNodeType<CalibrateFieldCoordinateAction>("CalibrateFieldCoordinates", node);

        tree = factory.createTreeFromFile("config/behavior_trees/main_tree.xml");
        // Create service to start the behavior tree
        start_service_ = node->create_service<std_srvs::srv::Trigger>("start_autonomy",
                                                                      std::bind(&BehaviorTreeExecutor::startCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Create service to cancel the behavior tree
        cancel_service_ = node->create_service<std_srvs::srv::Trigger>("cancel_autonomy",
                                                                       std::bind(&BehaviorTreeExecutor::cancelCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    std::shared_ptr<rclcpp::Node> node;
private:
    void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (running_)
        {
            response->success = false;
            response->message = "Behavior tree already running";
            return;
        }

        // Start behavior tree in a separate thread
        running_ = true;
        bt_thread_ = std::thread([this]()
                                 { 
            while (running_ && rclcpp::ok()) {
                auto status = tree.tickExactlyOnce(); 
                // Check if the tree has completed 
                if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) { 
                    RCLCPP_INFO(node->get_logger(), "Behavior tree finished with status: %s", status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE"); 

                    running_ = false; 

                    break; 
                } // Sleep to avoid hogging CPU 

                std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
            } });

        response->success = true;

        response->message = "Behavior tree started";
    }

    void cancelCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!running_)
        {
            response->success = false;
            response->message = "No behavior tree running";
            return;
        }
        // Set flag to stop the execution
        running_ = false;
        // Wait for the thread to complete
        if (bt_thread_.joinable())
        {
            bt_thread_.join();
        }
        // Reset the tree
        tree.haltTree();
        response->success = true;
        response->message = "Behavior tree canceled";
    }

    BT::BehaviorTreeFactory factory;
    BT::Tree tree;
    std::thread bt_thread_;
    bool running_ = false; // This is unsafe
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    BehaviorTreeExecutor t;
    rclcpp::spin(t.node);
    rclcpp::shutdown();
    return 0;
}