#include "rovr_interfaces/action/go_to_dig_location.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using GoToDigLocation = rovr_interfaces::action::GoToDigLocation;
using namespace BT;

class GoToDigLocationAction : public RosActionNode<GoToDigLocation>
{
public:
  GoToDigLocationAction(
      const std::string &instance_name,
      const NodeConfig &conf)
      : RosActionNode<GoToDigLocation>(
            instance_name,
            conf,
            RosNodeParams(std::make_shared<rclcpp::Node>("go_to_dig_location_client"), "go_to_dig_location"))
  {
  }

  bool setGoal(__attribute__ ((unused)) Goal &goal) override
  {
    return true;
  }

  NodeStatus onResultReceived(__attribute__ ((unused)) const WrappedResult &result) override
  {
    return NodeStatus::SUCCESS;
  }
};
