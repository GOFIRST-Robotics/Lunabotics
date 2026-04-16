#include <functional>
#include <mutex>
#include <memory>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/plugins.hpp"

// BT Nodes
#include "behavior_control/log_node.hpp"
#include "behavior_control/is_button_pressed_node.hpp"
#include "behavior_control/is_button_just_pressed_node.hpp"

// BT Service Nodes
#include "behavior_control/set_motor_velocity_node.hpp"
#include "behavior_control/set_motor_duty_cycle_node.hpp"

#include "behavior_control/get_motor_property_nodes.hpp"

// BT Action Nodes
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
        this->initalize_parameter();

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

    void init_factory() {
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

        // Setup Motor Control Actions
        
        // Set Motor Commands
        BT::RosNodeParams motor_params;
        motor_params.nh = shared_from_this();
        motor_params.default_port_value = "motor/set"; // The service name

        factory.registerBuilder<SetMotorVelocity>(
            "SetMotorVelocity",
            [motor_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<SetMotorVelocity>(name, config, motor_params);
            }
        );

        factory.registerBuilder<SetMotorDutyCycle>(
            "SetMotorDutyCycle",
            [motor_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<SetMotorDutyCycle>(name, config, motor_params);
            }
        );

        // Get Motor Commands
        BT::RosNodeParams get_motor_params;
        get_motor_params.nh = shared_from_this();
        get_motor_params.default_port_value = "motor/get"; // The service name in your MotorControlNode

        factory.registerBuilder<GetMotorCurrent>("GetMotorCurrent", 
            [get_motor_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<GetMotorCurrent>(name, config, get_motor_params);
            });

        factory.registerBuilder<GetMotorVelocity>("GetMotorVelocity", 
            [get_motor_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<GetMotorVelocity>(name, config, get_motor_params);
            });

        factory.registerBuilder<GetMotorDutyCycle>("GetMotorDutyCycle", 
            [get_motor_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<GetMotorDutyCycle>(name, config, get_motor_params);
            });

        factory.registerBuilder<GetMotorPosition>("GetMotorPosition", 
            [get_motor_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<GetMotorPosition>(name, config, get_motor_params);
            });

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
    }

private:
    BT::BehaviorTreeFactory factory;
    BT::Blackboard::Ptr blackboard;
    rclcpp::WallRate::SharedPtr loop_rate;
    rclcpp_action::Server<BehaviorControl>::SharedPtr action_server;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<rovr_interfaces::msg::StreamDeckState>::SharedPtr stream_deck_sub;

    std::mutex blackboard_mutex;

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
        
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("behavior_control");
        std::string behavior_tree_path = package_share_directory + "/testing_tree.xml";
        
        auto current_tree = factory.createTreeFromFile(behavior_tree_path, this->blackboard);

        rclcpp::WallRate loop_rate(100ms);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;

        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
            if (goal_handle->is_canceling()) {
                current_tree.haltTree(); // Stop the motors!
                auto result = std::make_shared<BehaviorControl::Result>();
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            { // MUTEX BRACKETS
                std::lock_guard<std::mutex> lock(blackboard_mutex);
                status = current_tree.tickOnce();
            }

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
        std::lock_guard<std::mutex> lock(blackboard_mutex);
        this->blackboard->set("joy_message", *msg);
    }

    void stream_deck_callback(const rovr_interfaces::msg::StreamDeckState::SharedPtr msg) {
        sensor_msgs::msg::Joy virtual_joy;
        virtual_joy.header.stamp = this->now();
        
        for (bool state : msg->button_states) {
            virtual_joy.buttons.push_back(state ? 1 : 0);
        }
        // Just to be safe, provide an empty axes vector
        virtual_joy.axes.resize(0);

        std::lock_guard<std::mutex> lock(blackboard_mutex);
        this->blackboard->set("stream_deck_message", virtual_joy);
    }

    void initalize_parameter() {
        // We allow "undeclared" parameters under these prefixes by using Descriptor
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.dynamic_typing = true;

        this->declare_parameters("buttons", std::map<std::string, int>{});
        this->declare_parameters("axes", std::map<std::string, int>{});
        this->declare_parameters("streamdeck", std::map<std::string, int>{});
        this->declare_parameters("hardware", std::map<std::string, int>{});
    }

    void setup_blackboard() {
        std::lock_guard<std::mutex> lock(blackboard_mutex);
        // Buttons
        std::map<std::string, int> all_buttons;
        this->get_parameters_by_prefix("buttons", all_buttons);
        for (auto const& [name, val] : all_buttons) {
            this->blackboard->set(name, val);
        }

        // Axes
        std::map<std::string, int> all_axes;
        this->get_parameters_by_prefix("axes", all_axes);
        for (auto const& [name, val] : all_axes) {
            this->blackboard->set(name, val);
        }

        // StreamDeck
        std::map<std::string, int> all_stream_deck;
        this->get_parameters_by_prefix("streamdeck", all_stream_deck);
        for (auto const& [name, val] : all_stream_deck) {
            this->blackboard->set("SD_" + name, val);
        }

        // Load Hardware IDs
        std::map<std::string, int> all_hardware;
        this->get_parameters_by_prefix("hardware", all_hardware);
        for (auto const& [name, id] : all_hardware) {
            this->blackboard->set("HW_" + name, id);
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(BehaviorControlActionServer)