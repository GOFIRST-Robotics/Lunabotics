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

  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setGoal(RosActionNode::Goal& goal) override 
  {
    return true;
  }

  NodeStatus onResultReceived(const WrappedResult &result) override
  {
    // Someone double check that this makes sense pls :)
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Calibration succeeded.");
      result.result->success = true;
      return NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Calibration aborted.");
      result.result->success = false;
      return NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "Calibration canceled.");
      result.result->success = false;
      return NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
      return NodeStatus::FAILURE;
    }
  }
};
