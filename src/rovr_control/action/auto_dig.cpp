// following the theoretically working template in calibrate_field_coordinate.cpp so that file has the explanations for the lines

#include "rovr_interfaces/action/auto_dig.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

using AutoDig = rovr_interaces::action::AutoDig

class AutoDigAction : public BT::RosActionNode<AutoDig> {
    public:
        AutoDigAction(
            const std::string& name,
            const BT::NodeConfig& conf,
            const BT::RosNodeParams& params
        )
        : BT::RosActionNode<AutoDig>(name, conf, params)
        {}

        bool setGoal(BT::RosActionNode<CalibrateFieldCoordinate>::Goal& goal) override {
            return true;
        }

        BT::NodeStatus onResultReceived(const WrappedResult& wr) override {
            return (wr.result->success) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

};

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    auto node = std::make_shared<rclcpp::Node>("auto_dig_action_client");

    BT::RosNodeParams params;
    params.nh = node;
    params.default_port_value = "auto_dig";
    factory.registerNodeType<AutoDigAction>("AutoDig", params);

    rclcpp::shutdown();
    return 0;
}