//Someone please double check this is correct
#include "rovr_interfaces/action/auto_offload.hpp"
//#include "nav2_behavior_tree/bt_action_node.hpp"
//#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

using AutoOffload = rovr_interfaces::action::AutoOffload;

class AutoOffloadAction : public nav2_behavior_tree::RosActionNode<AutoOffload>{
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

//BT_REGISTER_NODES(factory)
//{
 //   BT::NodeBuilder builder =
 //   [](const std::string & name, const BT::NodeConfiguration & config)
 //   {
 //       return std::make_unique<AutoOffload>(name, "auto_offload", config)
 //   };

 //   factory.registerBuilder<AutoOffload>("AutoOffload", builder);
//}

//main method for the node
int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoOffloadAction>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}