#include <functional>
#include <memory>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/plugins.hpp"

#include "behavior_control/log_node.hpp"
#include "behavior_control/calibrate_feild_coordinates_node.hpp"
#include "behavior_control/dig_location_node.hpp"

#include "rovr_interfaces/action/behavior_control_tree.hpp"

class BehaviorControlActionServer : public rclcpp::Node {
public:
    using BehaviorControlTree = rovr_interfaces::action::BehaviorControlTree;
    using BehaviorControlTreeGoalHandle = rclcpp_action::ServerGoalHandle<BehaviorControlTree>;

    explicit BehaviorControlActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("behavior_control_tree_action_server", options) {
        this->action_server = rclcpp_action::create_server<BehaviorControlTree>(
            this,
            "behavior_control_tree",
            std::bind(&BehaviorControlActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BehaviorControlActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&BehaviorControlActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }

    void setup_tree() {
        // Setup Groot2 Behavior Tree
        BT::BehaviorTreeFactory factory;
        
        // Setup Log String Tree Node
        factory.registerBuilder<LogString>(
            "LogString",
            [this](const std::string& name, const BT::NodeConfiguration& config) { 
                return std::make_unique<LogString>(name, config, this->get_logger()); 
            }
        );

        // Setup Calibrate Field Coordinates Action
        BT::RosNodeParams calibrate_field_coordinates_params;
        calibrate_field_coordinates_params.nh = shared_from_this();
        calibrate_field_coordinates_params.default_port_value = "calibrate_field_coordinates";
        factory.registerBuilder<CalibrateFieldCoordinatesAction>(
            "CalibrateFieldCoordinates",
            [calibrate_field_coordinates_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<CalibrateFieldCoordinatesAction>(name, config, calibrate_field_coordinates_params);
            }
        );

        // Setup Dig Location Action
        BT::RosNodeParams dig_location_params;
        dig_location_params.nh = shared_from_this();
        dig_location_params.default_port_value = "dig_location_server";
        factory.registerBuilder<DigLocationAction>(
            "DigLocation",
            [dig_location_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<DigLocationAction>(name, config, dig_location_params);
            }
        );

        // Load behavior tree from Groot2
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("behavior_control");
        std::string behavior_tree_path = package_share_directory + "/testing_tree.xml";
        this->tree = factory.createTreeFromFile(behavior_tree_path);
    }

private:
    BT::Tree tree;
    rclcpp::WallRate::SharedPtr loop_rate;
    rclcpp_action::Server<BehaviorControlTree>::SharedPtr action_server;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BehaviorControlTree::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<BehaviorControlTreeGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<BehaviorControlTreeGoalHandle> goal_handle) {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&BehaviorControlActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<BehaviorControlTreeGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing Behavior Control Tree");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<BehaviorControlTree::Feedback>();
        auto result = std::make_shared<BehaviorControlTree::Result>();

        // Initalize and resets the tree on a new execute
        setup_tree();

        rclcpp::WallRate loop_rate(std::chrono::milliseconds(100));
        BT::NodeStatus status = BT::NodeStatus::RUNNING;

        // Run Tree
        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
            if (goal_handle->is_canceling()) {
                tree.haltTree(); // Crucial: Stop all running BT nodes
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Behavior Tree Action Canceled");
                return;
            }

            status = tree.tickOnce();

            // Feedback (can be made more complex)
            feedback->current_status = BT::toStr(status);
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }

        // Final Result
        if (status == BT::NodeStatus::SUCCESS) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Behavior Tree Action Completed: SUCCESS");
        } else {
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Behavior Tree Action Completed: FAILURE");
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(BehaviorControlActionServer)