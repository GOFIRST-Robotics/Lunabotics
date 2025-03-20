// following the theoretically working template in calibrate_field_coordinate.cpp so that file has the explanations for the lines

#include "rovr_interfaces/action/auto_dig.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using AutoDig = rovr_interaces::action::AutoDig;

class AutoDigAction : public BT::RosActionNode<AutoDig> {
    public:
        AutoDigAction(
            const std::string& instance_name,
            const BT::NodeConfig& conf,
            const BT::RosNodeParams& params
        )
        : BT::RosActionNode<AutoDig>(instance_name, conf, params) {}

        BT::NodeStatus onResultReceived(const WrappedResult& result) override {
          //TODO: implement this :)
          return BT::NodeStatus::SUCCESS;
      }

      bool setGoal(Goal& goal) override {
          //TODO: implement this :)
          return true;
      }

};

//BT_REGISTER_NODES(factory)
//{
  //  BT::NodeBuilder builder =
   // [](const std::string & name, const BT::NodeConfiguration & config)
    //{
     // return std::make_unique<AutoDig>(name, "auto_dig", config);
    //};

  //factory.registerBuilder<AutoDigAction>("AutoDig", builder);
//}

// Main method for the node
int main(int argc, char**argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("auto_dig_node");

    BT::NodeConfig config;
    BT::RosNodeParams ros_node_params;
    ros_node_params.nh = node;

    // Create an instance of your action class, passing the ROS node
    auto action_node = std::make_shared<CalibrateFieldCoordinateAction>("auto_dig_action", config, ros_node_params);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}