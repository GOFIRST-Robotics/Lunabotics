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

class MyActionServer : public TreeExecutionServer
{
public:
    MyActionServer(const rclcpp::NodeOptions &options) : TreeExecutionServer(options)
    {
    }
    void registerNodesIntoFactory(BT::BehaviorTreeFactory &factory)
    {
        factory.registerNodeType<AutoOffloadAction>("AutoOffload", this->node());
        factory.registerNodeType<GoToDigLocationAction>("GoToDigLocation", this->node());
        factory.registerNodeType<AutoDigAction>("AutoDig", this->node());
        factory.registerNodeType<CalibrateFieldCoordinateAction>("CalibrateFieldCoordinates", this->node());
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