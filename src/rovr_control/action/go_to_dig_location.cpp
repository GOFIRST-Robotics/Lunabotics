#include "rovr_interfaces/action/go_to_dig_location.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using GoToDigLocation = rovr_interfaces::action::GoToDigLocation;

class GoToDigLocationAction: public BT::RosActionNode<GoToDigLocation> {
public:
GoToDigLocation(
    const std::string& instance_name, 
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosActionNode<GoToDigLocation>(instance_name, conf, params) {}

    BT::NodeStatus onResultReceived(const WrappedResult& result) override {
      //TODO: implement this :)
      return BT::NodeStatus::SUCCESS;
  }

  bool setGoal(Goal& goal) override {
      //TODO: implement this :)
      return true;
  }
};



// Main method for the node
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); 

  // Create a ROS node
  auto node = std::make_shared<rclcpp::Node>("go_to_dig_location_node");

  BT::NodeConfig config;
  BT::RosNodeParams ros_node_params;
  ros_node_params.nh = node;

  // Create an instance of your action class, passing the ROS node
  auto action_node = std::make_shared<CalibrateFieldCoordinateAction>("go_to_dig_location", config, ros_node_params);

  rclcpp::spin(node);
  rclcpp::shutdown(); 
  return 0;
}