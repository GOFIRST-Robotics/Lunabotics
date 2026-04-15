#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "rclcpp/rclcpp.hpp"
// https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MoveToAction : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {
public:
    MoveToAction(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) 
    : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, config, params) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal") };
    }
    
    bool setGoal(Goal& goal) override {
        geometry_msgs::msg::PoseStamped target_pose;
        if (!getInput("goal", target_pose)) {
            auto node_ptr = node_.lock();
            RCLCPP_ERROR(node_ptr->get_logger(), "[%s]: Goal port is empty!", name().c_str());
            return false; 
        }

        goal.pose = target_pose;
        
        // Ensure the timestamp is current so Nav2 doesn't reject it for being "in the past"
        auto node_ptr = node_.lock();
        if (node_ptr) {
            goal.pose.header.stamp = node_ptr->now();
        }
        
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& result) override {
        auto node_ptr = node_.lock();
        // TODO: Better handle all the results returned for NavigateToPose
        if (!node_ptr) return BT::NodeStatus::FAILURE;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_ptr->get_logger(), "[%s]: Reached destination.", name().c_str());
                return BT::NodeStatus::SUCCESS;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_ptr->get_logger(), "[%s]: Navigation aborted.", name().c_str());
                return BT::NodeStatus::FAILURE;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(node_ptr->get_logger(), "[%s]: Navigation canceled.", name().c_str());
                return BT::NodeStatus::FAILURE;
            default:
                return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) {
        // TODO: Better handle and show feedback
        
        return NodeStatus::RUNNING;
    }
};