//#include "behaviortree_cpp/action_node.h"
#include "/workspaces/Lunabotics/src/behaviortree_ros2/behaviortree_ros2/include/behaviortree_ros2/bt_action_node.hpp" 

class AutoDigAction : public BT::RosActionNode<AutoDig> {
    public:
        AutoDigAction(
            const std::string& name,
            const BT::NodeConfig& conf,
            const BT::RosNodeParams& params
        )
        : BT::RosActionNode<AutoDig>(name, conf, params)
        {}

        void AutoDigAction::on_tick() {
            if (!BT::isStatusActive(status())) {
                //call the auto_dig_server.py file somehow
            }

        }   

        BT::NodeStatus onResultReceived(const BT::RosActionNode<rovr_interfaces::action::AutoDig>::WrappedResult& wr) override {   //might not need override, depends if class definition line inheritance works
            if (wr.result->success) {
                return BT::NodeStatus::SUCCESS;
            }
            
            return BT::NodeStatus::FAILURE;
        }

};