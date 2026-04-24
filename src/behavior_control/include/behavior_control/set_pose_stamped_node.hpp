#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class SetPoseStamped : public BT::SyncActionNode {
public:
    SetPoseStamped(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Logger node_logger)
    : BT::SyncActionNode(name, config), logger(node_logger) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("yaw", "Rotation around the Z axis in degrees"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")
        };
    }

    BT::NodeStatus tick() override {
        double x, y, yaw_degrees;
        if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw_degrees)) {
            RCLCPP_ERROR(logger, "SetPoseStamped: One or more input ports are empty or not mapped!");
            return BT::NodeStatus::FAILURE;
        }

        // Construct PoseStamped message
        geometry_msgs::msg::PoseStamped pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = rclcpp::Clock().now();


        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = 0.0;
        
        // Convert yaw to quaternion
        double yaw_radians = yaw_degrees * (M_PI / 180.0);

        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw_radians);
        pose_msg.pose.orientation = tf2::toMsg(quat);

        // Set the output port
        setOutput("pose", pose_msg);

        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Logger logger;
};