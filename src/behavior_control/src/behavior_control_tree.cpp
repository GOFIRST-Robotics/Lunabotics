#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/plugins.hpp"

#include "behavior_control/log_node.hpp"
#include "behavior_control/calibrate_feild_coordinates_node.hpp"
#include "behavior_control/dig_location_node.hpp"

class BehaviorControlTreeNode : public rclcpp::Node {
public:
    BehaviorControlTreeNode(const std::string& node_name) 
    : Node(node_name) {
        // Create Timer
        timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BehaviorControlTreeNode::behavior_tree_callback, this)
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
        std::string behavior_tree_path = package_share_directory + "/main_tree.btproj";
        tree = factory.createTreeFromFile(behavior_tree_path);
    }
private:
    void behavior_tree_callback() {
        // Tree node logic
        BT::NodeStatus status = tree.tickOnce();

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Tree Ended: SUCCESS");
            timer->cancel();
            rclcpp::shutdown();
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "Tree Ended: FAILURE");
            timer->cancel();
            rclcpp::shutdown();
        }
    }

    BT::Tree tree;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BehaviorControlTreeNode>("behavior_control_tree_node");

    node->setup_tree();

    rclcpp::spin(node);    
    rclcpp::shutdown();

    return 0;
}