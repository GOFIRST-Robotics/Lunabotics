#include "rovr_interfaces/action/go_to_dig_location.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using GoToDigLocation = rovr_interfaces::action::GoToDigLocation;

class GoToDigLocationAction: public BT::RosActionNode<GoToDigLocation> {
  public:
      GoToDigLocationAction(
        const std::string& instance_name, 
        const BT::NodeConfig& conf)

      : BT::RosActionNode<GoToDigLocation>(
          instance_name,
          conf,
          BT::RosNodeParams(std::make_shared<rclcpp::Node>("go_to_location_node"))) {}

      static BT::PortsList providedPorts() {
          return providedBasicPorts({});
      }

      BT::NodeStatus onResultReceived(const WrappedResult& result) override {
        //TODO: implement this :)
        return BT::NodeStatus::SUCCESS;
      }

      bool setGoal(Goal& goal) override {
      //TODO: implement this :)
      return true;
      }
};
