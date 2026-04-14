#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using CalibrateFieldCoordinates = rovr_interfaces::action::CalibrateFieldCoordinates;
using namespace BT;

class CalibrateFieldCoordinateAction : public RosActionNode<CalibrateFieldCoordinates>
{
public:
    static BT::PortsList providedPorts()
    {
        return
        {
            BT::InputPort<std::string>("action_name")
        };
    }
    CalibrateFieldCoordinateAction(const std::string &name, const BT::NodeConfig &conf,
                                   const BT::RosNodeParams &params)
        : RosActionNode<CalibrateFieldCoordinates>(name, conf, params)
    {
    }

    bool setGoal(dGoal &goal) override
    {
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult &result) override
    {
        //result is gotten from goal_handler_->get_result() in bt_action_node.hpp.
        //It contains the result code and the result message from the action server

        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // The action server completed the calibrated field coordinates successfully
                return NodeStatus::SUCCESS;
            case rclcpp_action::ResultCode::ABORTED:
                // Something went wrong (eg the calibration failed or a sensor failed)
                return NodeStatus::FAILURE;
            case rclcpp_action::ResultCode::CANCELED:
                // The action was canceled
                return NodeStatus::CANCELED;
            default:
                // Any other weirdness should generally be a failure
                return NodeStatus::FAILURE;
        }
    }
};