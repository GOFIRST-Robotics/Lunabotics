#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "action_msgs/srv/cancel_goal.hpp"

using namespace BT;

class CancelActionNode : public BT::SyncActionNode
{
public:
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("action_name"),
        };
    }

    CancelActionNode(const std::string &name, const BT::NodeConfig &conf,
                     rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, conf), node_(node)
    {
    }

    BT::NodeStatus tick() override
    {
        std::string action_name;
        if (!getInput("action_name", action_name))
        {
            RCLCPP_ERROR(node_->get_logger(), "CancelActionNode: missing required input 'action_name'");
            return BT::NodeStatus::FAILURE;
        }

        // The ROS2 action cancel service is always at /<action_name>/_action/cancel_goal
        std::string service_name = action_name + "/_action/cancel_goal";

        auto client = node_->create_client<action_msgs::srv::CancelGoal>(service_name);

        if (!client->wait_for_service(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "CancelActionNode: cancel service '%s' not available",
                         service_name.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

        auto future = client->async_send_request(request);
        auto deadline = node_->now() + rclcpp::Duration::from_seconds(3.0);
        while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
        {
            if (node_->now() >= deadline)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "CancelActionNode: timed out waiting for cancel response on '%s'",
                             service_name.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        auto response = future.get();
        if (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "CancelActionNode: successfully canceled all goals on '%s'",
                        action_name.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "CancelActionNode: cancel returned code %d on '%s'",
                        response->return_code, action_name.c_str());
            return (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_REJECTED)
                       ? BT::NodeStatus::SUCCESS
                       : BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};