#include "rovr_interfaces/action/coord_return.hpp"

#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using CoordReturn = rovr_interfaces::action::ReturnToCoordinate;
using namespace BT;

class CoordReturnAction : public RosActionNode<CoordReturn>
{
    public:
       static BT::PortsList providedPorts() 
       {
              return
              {
                BT::InputPort<std::string>("action_name"),
                BT::InputPort<double>("x_pos"),
                BT::InputPort<double>("y_pos")
              };
        }

    CoordReturnAction(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
    : RosActionNode<CoordReturn>(name, conf, params)
    {
        // Initialize any member variables or state here
    }

    // attribute__((unused)) is used to suppress compiler warnings about unused parameters
    bool setGoal(Goal &goal) override
    {
        bool x_pos_success = getInput<double>("x_pos", goal.x_pos);
        bool y_pos_success = getInput<double>("y_pos", goal.y_pos);
        return x_pos_success && y_pos_success;
    }

    NodeStatus onResultReceived(const WrappedResult &result) override
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // The action server completed the return to coordinate successfully
                return NodeStatus::SUCCESS;
            case rclcpp_action::ResultCode::ABORTED:
                // Something went wrong (eg the robot got stuck or a sensor failed)
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

