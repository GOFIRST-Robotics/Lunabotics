//Someone please double check this is correct
#include "rovr_interfaces/action/auto_offload.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using AutoOffload = rovr_interfaces::action::AutoOffload;

class AutoOffloadAction : public BT::RosActionNode<AutoOffload>{
    public:
        AutoOffloadAction(
            const std::string& instance_name,
            const BT::NodeConfig& conf,
            const RosNodeParams& params)
        : BT::RosActionNode<AutoOffload>(instance_name, conf, params){}

        BT::NodeStatus onResultReceived(const WrappedResult& result) override {
            return BT::NodeStatus::SUCCESS;
        }

        bool setGoal(Goal& goal)override {
            return true;
        }
        
        
};



//main method for the node
int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoOffloadAction>();

    BT::NodeConfig config;
    BT::RosNodeParams params;
    ros_node_params.nh = node;

    auto action_node = std::make_shared<AutoOffload>("auto_offload_action", config, ros_node_params);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}