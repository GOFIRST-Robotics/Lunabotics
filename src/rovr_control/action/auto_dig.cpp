// following the theoretically working template in calibrate_field_coordinate.cpp so that file has the explanations for the lines

#include "rovr_interfaces/action/auto_dig.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"

using AutoDig = rovr_interaces::action::AutoDig;

class AutoDigAction : public nav2_behavior_tree::BtActionNode<AutoDig> {
    public:
        AutoDigAction(
            const std::string & xml_tag_name,
            const BT::string & action_name,
            const BT::NodeConfiguration & conf
        )
        : nav2_behavior_tree::BtActionNode<AutoDig>(xml_tag_name, action_name, conf)
        {}

        //bool setGoal(BT::RosActionNode<CalibrateFieldCoordinate>::Goal& goal) override {
        //    return true;
        // }

       // BT::NodeStatus onResultReceived(const WrappedResult& wr) override {
       //     return (wr.result->success) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        // }

};

BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<AutoDig>(name, "auto_dig", config);
    };

  factory.registerBuilder<AutoDigAction>("AutoDig", builder);
}

// Main method for the node
int main(int argc, char**argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    rclcpp::shutdown();
    return 0;
}