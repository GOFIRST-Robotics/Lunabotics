//Someone please double check this is correct
#include "behaviortree_cpp/action_node.h"
#include "/workspaces/Lunabotics/src/behaviortree_ros2/behaviortree_ros2/include/behaviortree_ros2/bt_action_node.hpp"


class AutoOffloadAction : public BT ::RosActionNode<AutoOffload>{
    public:
        AutoOffloadAction(
            const std::string& name,
            const BT::NodeConfig& conf,
            const BT::RosNodeParams& params
        )
        : BT::RosActionNode<AutoOffload>(name, conf, params) {}

        bool setGoal(BT::RosActionNode<AutoOffload>::Goal& goal) override {
            return true;
        }

        BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
            return (wr.result->success) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
};

//main method for the node
int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    auto node = std::make_shared<rclcpp::Node>("auto_offload_action_client");

    // provide the ROs node and the name of the action service
    BT::RosNodeParams params;
    params.nh = node;
    params.default_port_value = "auto_offload";
    factory.registerNodeType<AutoOffloadAction>("AutoOffload", params);

    // Free up any resources being used by the node
    rclcpp::shutdown();
    return 0;
}