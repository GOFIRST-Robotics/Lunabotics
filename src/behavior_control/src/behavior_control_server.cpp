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
#include "behavior_control/is_button_pressed_node.hpp"
#include "behavior_control/is_button_just_pressed_node.hpp"

#include "behavior_control/calibrate_feild_coordinates_node.hpp"
#include "behavior_control/dig_location_node.hpp"
#include "behavior_control/move_to_node.hpp"

#include "rovr_interfaces/action/behavior_control.hpp"
#include "rovr_interfaces/msg/stream_deck_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

class BehaviorControlActionServer : public rclcpp::Node {
public:
    using BehaviorControl = rovr_interfaces::action::BehaviorControl;
    using BehaviorControlGoalHandle = rclcpp_action::ServerGoalHandle<BehaviorControl>;

    explicit BehaviorControlActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("behavior_control_tree_action_server", options) {
        this->blackboard = BT::Blackboard::create();

        this->action_server = rclcpp_action::create_server<BehaviorControl>(
            this,
            "behavior_control_tree",
            std::bind(&BehaviorControlActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BehaviorControlActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&BehaviorControlActionServer::handle_accepted, this, std::placeholders::_1)
        );

        this->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 
            10, 
            std::bind(&BehaviorControlActionServer::joy_callback, this, std::placeholders::_1)
        );

        this->stream_deck_sub = this->create_subscription<rovr_interfaces::msg::StreamDeckState>(
            "control/stream_deck", 
            10, 
            std::bind(&BehaviorControlActionServer::stream_deck_callback, this, std::placeholders::_1)
        );
    }

    void setup_tree() {
        // Setup Groot2 Behavior Tree
        BT::BehaviorTreeFactory factory;
        
        // Setup Button Press Nodes
        factory.registerNodeType<IsButtonJustPressed>("IsButtonJustPressed");
        factory.registerNodeType<IsButtonPressed>("IsButtonPressed");

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

        // Setup Move To Action
        BT::RosNodeParams move_to_params;
        move_to_params.nh = shared_from_this();
        move_to_params.default_port_value = "navigate_to_pose"; // Nav2 Action Server
        factory.registerBuilder<MoveToAction>(
            "MoveTo",
            [move_to_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<MoveToAction>(name, config, move_to_params);
            }
        );

        this->setup_blackboard();

        // Load behavior tree from Groot2
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("behavior_control");
        std::string behavior_tree_path = package_share_directory + "/testing_tree.xml";
        this->tree = factory.createTreeFromFile(behavior_tree_path, this->blackboard);
    }

private:
    BT::Tree tree;
    BT::Blackboard::Ptr blackboard;
    rclcpp::WallRate::SharedPtr loop_rate;
    rclcpp_action::Server<BehaviorControl>::SharedPtr action_server;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<rovr_interfaces::msg::StreamDeckState>::SharedPtr stream_deck_sub;

    // Handle inital request
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BehaviorControl::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle tree cancellation
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<BehaviorControlGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel Behavior Control Server");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // On handle goal after request acceptance
    void handle_accepted(const std::shared_ptr<BehaviorControlGoalHandle> goal_handle) {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&BehaviorControlActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Execute behavior tree
    void execute(const std::shared_ptr<BehaviorControlGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing Behavior Control Tree");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<BehaviorControl::Feedback>();
        auto result = std::make_shared<BehaviorControl::Result>();

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

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        this->blackboard->set("joy_message", *msg);
    }

    void stream_deck_callback(const rovr_interfaces::msg::StreamDeckState::SharedPtr msg) {
        sensor_msgs::msg::Joy virtual_joy;
    
        // Convert the bool[6] array to the int vector Joy expects
        for (bool state : msg->button_states) {
            virtual_joy.buttons.push_back(state ? 1 : 0);
        }

        // Update the blackboard
        this->blackboard->set("stream_deck_message", virtual_joy);
    }

    void setup_blackboard() {
        // Buttons
        std::vector<std::string> button_names = {
            "X_BUTTON", "A_BUTTON", "B_BUTTON", "Y_BUTTON", "LEFT_BUMPER", 
            "RIGHT_BUMPER", "START_BUTTON", "BACK_BUTTON"
        };
        for (const auto& name : button_names) {
            int val;
            if (this->get_parameter("buttons." + name, val)) {
                this->blackboard->set(name, val);
            }
        }

        // Axes
        std::vector<std::string> axes_names = {
            "LEFT_JOYSTICK_HORIZONTAL", "LEFT_JOYSTICK_VERTICAL",
            "RIGHT_JOYSTICK_HORIZONTAL", "RIGHT_JOYSTICK_VERTICAL"
        };
        for (const auto& name : axes_names) {
            int val;
            if (this->get_parameter("axes." + name, val)) {
                this->blackboard->set(name, val);
            }
        }

        // StreamDeck
        std::vector<std::string> sd_names = {
            "START_AUTO", "AUTO_DIG", "AUTO_DUMP", 
            "APRILTAG_DETECT", "GO_TO_DIG_SITE", "ESTOP"
        };
        for (const auto& name : sd_names) {
            int val;
            if (this->get_parameter("streamdeck." + name, val)) {
                this->blackboard->set(name, val);
            }
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(BehaviorControlActionServer)