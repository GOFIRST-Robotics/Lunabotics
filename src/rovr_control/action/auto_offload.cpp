//Someone please double check this is correct
#include "behaviortree_cpp/action_node.h"
#include "/workspaces/Lunabotics/src/behaviortree_ros2/behaviortree_ros2/include/behaviortree_ros2/bt_action_node.hpp"


class AutoOffloadAction : public BT ::RosActionNode<AutoOffload>{
    public:
        AutoOffloadAction(
            const std::string& name,
            const BT::NodeConfig& conf,
            const BT::RosNodeParams& params
        )
        : BT::RosActionNode<AutoOffload>(name, conf, params)
        {}

        BT::NodeStatus onResultReceived(const BT::RosActionNode<rovr_interfaces::action::AutoOffload>::WrappedResult& wr) override{
            if (wr.resultt->success){
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::FAILURE;
        }
};