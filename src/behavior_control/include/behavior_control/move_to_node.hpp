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
        return { BT::InputPort<geometry_msgs::action::PoseStamped>("goal") };
    }
    
    bool setGoal(Goal& goal) override {
        getInput("goal", goal.goal)
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& result) override {
        auto node_ptr = node_.lock();
        // TODO: Better handle all the results returned for NavigateToPose
        if (node_ptr) {
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_WARN(node_ptr->get_logger(), "[%s]: Move to pose failed.", name().c_str());
                return BT::NodeStatus::FAILURE;
            }
            
            RCLCPP_INFO(node_ptr->get_logger(), "[%s]: Moved to pose successfully.", name().c_str());
        }

        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) {
        // TODO: Better handle and show feedback
        
        return NodeStatus::RUNNING;
    }
};