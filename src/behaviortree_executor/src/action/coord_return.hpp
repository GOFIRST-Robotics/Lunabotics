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
        return NodeStatus::SUCCESS;
    }

};

