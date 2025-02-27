//Someone please double check this is correct
#include "rovr_interfaces/action/auto_offload.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"

using AutoOffload = rovr_interfaces::action::AutoOffload;

class AutoOffloadAction : public nav2_behavior_tree::RosActionNode<AutoOffload>{
    public:
        AutoOffloadAction(
            const std::string & xml_tag_name,
            const std::string & action_name,
            const BT::NodeConfiguration & conf)
        : nav2_behavior_tree::BtActionNode<AutoOffload>(xml_tag_name, action_name, conf) {}
        
        //called when TreeNode is ticked, send request to action server
        //bool setGoal(BT::RosActionNode<AutoOffload>::Goal& goal) override {
           // return true;
        //}

        //BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
           // return (wr.result->success) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        //}
};

BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
        return std::make_unique<AutoOffload>(name, "auto_offload", config)
    };

    factory.registerBuilder<AutoOffload>("AutoOffload", builder);
}

//main method for the node
int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    rclcpp::shutdown();
    return 0;
}