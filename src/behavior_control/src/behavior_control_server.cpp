#include <functional>
#include <memory>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

// BT Nodes
#include "behavior_control/log_node.hpp"
#include "behavior_control/set_pose_stamped_node.hpp"
#include "behavior_control/is_button_pressed_node.hpp"
#include "behavior_control/is_button_just_pressed_node.hpp"

// BT Service Nodes
#include "behavior_control/set_motor_velocity_node.hpp"
#include "behavior_control/set_motor_duty_cycle_node.hpp"

#include "behavior_control/get_motor_property_nodes.hpp"

// BT Action Nodes
#include "behavior_control/calibrate_feild_coordinates_node.hpp"
#include "behavior_control/auto_offload_node.hpp"
#include "behavior_control/auto_dig_node.hpp"
#include "behavior_control/auto_dig_nav_offload_node.hpp"
#include "behavior_control/dig_location_node.hpp"
#include "behavior_control/move_to_node.hpp"

#include "rovr_interfaces/action/behavior_control.hpp"
#include "rovr_interfaces/msg/stream_deck_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

// NOTE: To test the behavior tree run
// ros2 action send_goal /behavior_control rovr_interfaces/action/BehaviorControl "{}"

class BehaviorControlActionServer : public rclcpp::Node {
public:
    using BehaviorControl = rovr_interfaces::action::BehaviorControl;
    using BehaviorControlGoalHandle = rclcpp_action::ServerGoalHandle<BehaviorControl>;

    explicit BehaviorControlActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("behavior_control_tree_action_server", options) {
        this->initalize_parameter();

        this->callback_group_subscribers_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        this->callback_group_actions_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_subscribers_;

        this->blackboard = BT::Blackboard::create();

        this->action_server = rclcpp_action::create_server<BehaviorControl>(
            this,
            "behavior_control",
            std::bind(&BehaviorControlActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BehaviorControlActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&BehaviorControlActionServer::handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            callback_group_actions_
        );

        this->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 
            1, 
            std::bind(&BehaviorControlActionServer::joy_callback, this, std::placeholders::_1),
            sub_options
        );

        this->stream_deck_sub = this->create_subscription<rovr_interfaces::msg::StreamDeckState>(
            "control/stream_deck", 
            1, 
            std::bind(&BehaviorControlActionServer::stream_deck_callback, this, std::placeholders::_1),
            sub_options
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

        // Setup Set Pose Stamped
        factory.registerBuilder<SetPoseStamped>(
            "SetPoseStamped",
            [this](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<SetPoseStamped>(name, config, this->get_logger());
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

        // Setup Auto Offload Action
        BT::RosNodeParams auto_offload_params;
        auto_offload_params.nh = shared_from_this();
        auto_offload_params.default_port_value = "auto_offload";
        factory.registerBuilder<AutoOffloadAction>(
            "AutoOffload",
            [auto_offload_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<AutoOffloadAction>(name, config, auto_offload_params);
            }
        );

        // Setup Auto Dig Action
        BT::RosNodeParams auto_dig_params;
        auto_dig_params.nh = shared_from_this();
        auto_dig_params.default_port_value = "auto_dig";
        factory.registerBuilder<AutoDigAction>(
            "AutoDig",
            [auto_dig_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<AutoDigAction>(name, config, auto_dig_params);
            }
        );

        // Setup Auto Dig Nav Offload Action
        BT::RosNodeParams auto_dig_nav_offload_params;
        auto_dig_nav_offload_params.nh = shared_from_this();
        auto_dig_nav_offload_params.default_port_value = "auto_dig_nav_offload";
        factory.registerBuilder<AutoDigNavOffloadAction>(
            "AutoDigNavOffload",
            [auto_dig_nav_offload_params](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<AutoDigNavOffloadAction>(name, config, auto_dig_nav_offload_params);
            }
        );

        // Setup Dig Location Action
        BT::RosNodeParams dig_location_params;
        dig_location_params.nh = shared_from_this();
        dig_location_params.default_port_value = "dig_location_server";
        factory.registerBuilder<DigLocationAction>(
            "GoToDigLocation",
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

    rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
    rclcpp::CallbackGroup::SharedPtr callback_group_actions_;

    std::unique_ptr<BT::Groot2Publisher> groot_publisher;

    // Handle inital request
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BehaviorControl::Goal> ) {
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

        std::string behavior_tree_file = this->get_parameter("behavior_tree_file").as_string();
        int groot_port = this->get_parameter("groot_publisher_port").as_int();
        long int tick_interval_ms = this->get_parameter("behavior_tree_tick_interval").as_int();
        
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("behavior_control");
        std::string behavior_tree_path = package_share_directory + behavior_tree_file;
        

        auto current_tree = factory.createTreeFromFile(behavior_tree_path, this->blackboard);
        groot_publisher = std::make_unique<BT::Groot2Publisher>(current_tree, groot_port);
        
        rclcpp::WallRate loop_rate{std::chrono::milliseconds(tick_interval_ms)};
        BT::NodeStatus status = BT::NodeStatus::RUNNING;

        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
            if (goal_handle->is_canceling()) {
                current_tree.haltTree(); // Stop the motors!
                auto result = std::make_shared<BehaviorControl::Result>();
                result->success = false;
                goal_handle->canceled(result);
                this->cleanup();
                return;
            }

            status = current_tree.tickOnce();

            // Feedback (can be made more complex)
            feedback->current_status = BT::toStr(status);
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }

        // Final Result
        if (status == BT::NodeStatus::SUCCESS) {
            result->success = true;
            goal_handle->succeed(result);
            this->cleanup();
            RCLCPP_INFO(this->get_logger(), "Behavior Tree Action Completed: SUCCESS");
        } else {
            result->success = false;
            goal_handle->abort(result);
            this->cleanup();
            RCLCPP_ERROR(this->get_logger(), "Behavior Tree Action Completed: FAILURE");
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
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

        this->blackboard->set("stream_deck_message", virtual_joy);
    }

    void initalize_parameter() {
        if (!this->has_parameter("groot_publisher_port")) {
            this->declare_parameter<int>("groot_publisher_port", 1667);
        }
        if (!this->has_parameter("behavior_tree_file")) {
            this->declare_parameter<std::string>("behavior_tree_file", "/tree/main_tree.xml");
        }
        if (!this->has_parameter("behavior_tree_tick_interval")) {
            this->declare_parameter<long int>("behavior_tree_tick_interval", 15);
        }
        
        // We allow "undeclared" parameters under these prefixes by using Descriptor
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.dynamic_typing = true;

        this->declare_parameter("buttons", rclcpp::ParameterType::PARAMETER_INTEGER);
        this->declare_parameter("axes", rclcpp::ParameterType::PARAMETER_INTEGER);
        this->declare_parameter("streamdeck", rclcpp::ParameterType::PARAMETER_INTEGER);
        this->declare_parameter("hardware", rclcpp::ParameterType::PARAMETER_INTEGER);
    }

    void setup_blackboard() {
        auto load_params = [this](const std::string& prefix, const std::string& bb_prefix) {
            std::map<std::string, rclcpp::Parameter> params;
            this->get_node_parameters_interface()->get_parameters_by_prefix(prefix, params);
            for (auto const& [name, val] : params) {
                if (val.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                    this->blackboard->set(bb_prefix + name, (int)val.as_int());
                }
            }
        };

        load_params("buttons", "");
        load_params("axes", "");
        load_params("streamdeck", "SD_");
        load_params("hardware", "HW_");

        this->debug_blackboard();
    }

    void debug_blackboard() {
        auto keys = this->blackboard->getKeys();
        RCLCPP_INFO(this->get_logger(), "--- Blackboard Contents ---");
        for (const auto& key : keys) {
            // Attempt to print as integer (since most of your params are ints)
            auto val = this->blackboard->get<int>(std::string(key));
            RCLCPP_INFO(this->get_logger(), "Key: %s | Value: %d", key.data(), val);
        }
        RCLCPP_INFO(this->get_logger(), "---------------------------");
    }

    void cleanup() {
        // Cleanup code if needed
        this->groot_publisher = nullptr;
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(BehaviorControlActionServer)

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;

    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<BehaviorControlActionServer>(options);
    node->init_factory();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}