#include "rclcpp/rclcpp.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

#include "action/calibrate_field_coordinates.hpp"
#include "action/auto_dig.hpp"
#include "action/auto_offload.hpp"
#include "action/go_to_dig_location.hpp"
#include "action/cancel_action.hpp"
#include "action/coord_return.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_ros2/tree_execution_server.hpp"


#include <thread>
#include <chrono>
#include <functional>

#include "std_srvs/srv/trigger.hpp"

class MyActionServer : public TreeExecutionServer
{
public:
    MyActionServer(const rclcpp::NodeOptions &options) : TreeExecutionServer(options)
    {
    }
    void registerNodesIntoFactory(BT::BehaviorTreeFactory &factory)
    {auto node_ptr = this->node();
        BT::RosNodeParams params;
        params.nh = this->node();
        
        factory.registerBuilder<AutoOffloadAction>("AutoOffload", 
        [params](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<AutoOffloadAction>(name, config, params);
        });

    factory.registerBuilder<GoToDigLocationAction>("GoToDigLocation", 
        [params](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<GoToDigLocationAction>(name, config, params);
        });

    factory.registerBuilder<AutoDigAction>("AutoDig", 
        [params](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<AutoDigAction>(name, config, params);
        });

    factory.registerBuilder<CalibrateFieldCoordinateAction>("CalibrateFieldCoordinates", 
        [params](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<CalibrateFieldCoordinateAction>(name, config, params);
        });

    factory.registerBuilder<CoordReturnAction>("ReturnToCoordinate", 
        [params](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<CoordReturnAction>(name, config, params);
        });

    factory.registerBuilder<CancelActionNode>("CancelAction", 
    [params](const std::string& name, const BT::NodeConfig& config) {
        // Lock the weak_ptr to get the SharedPtr
        auto node_ptr = params.nh.lock(); 
        if (!node_ptr) {
            throw std::runtime_error("ROS node pointer is no longer valid!");
        }
        return std::make_unique<CancelActionNode>(name, config, node_ptr);
    });
    }
};

int main(int argc, char **argv)
{
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