#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/include/behaviortree_ros2/bt_action_node.hpp"

using CalibrateFieldCoordinate = rovr_interfaces::action::CalibrateFieldCoordinates;

class CalibrateFieldCoordinateAction : public BT::RosActionNode<CalibrateFieldCoordinate> {
    public:
        CalibrateFieldCoordinateAction(
            const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params
        )
        : RosActionNode<CalibrateFieldCoordinate>(name, conf, params)
        {}

        BT::NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            if (wr.result->success) {
                return BT::NodeStatus::SUCCESS;
            }
            
            return BT::NodeStatus::FAILURE;
        }
};