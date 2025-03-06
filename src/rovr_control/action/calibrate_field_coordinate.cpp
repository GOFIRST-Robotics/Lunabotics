#include "rovr_interfaces/action/calibrate_field_coordinates.hpp"

// #include "nav2_behavior_tree/bt_action_node.hpp"

using CalibrateFieldCoordinate = rovr_interfaces::action::CalibrateFieldCoordinates;

// class CalibrateFieldCoordinateAction: public BT::

// class CalibrateFieldCoordinateAction: public nav2_behavior_tree::BtActionNode<CalibrateFieldCoordinate> {
// public:
//     CalibrateFieldCoordinateAction(
//         const std::string & xml_tag_name,
//         const std::string & action_name,
//         const BT::NodeConfiguration & conf)
//     : nav2_behavior_tree::BtActionNode<CalibrateFieldCoordinate>(xml_tag_name, action_name, conf) {}

//     // This is called when the TreeNode is ticked and it should
//     // send the request to the action server
//     // bool setGoal(BT::RosActionNode<CalibrateFieldCoordinate>::Goal& goal) override {
//     //     return true;
//     // }
    
//     // Callback executed when the reply is received.
//     // Based on the reply you may decide to return SUCCESS or FAILURE.
//     // BT::NodeStatus onResultReceived(const WrappedResult& wr) override {
//     //     return (wr.result->success) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
//     // }
// };

// BT_REGISTER_NODES(factory)
// {
//   BT::NodeBuilder builder =
//     [](const std::string & name, const BT::NodeConfiguration & config)
//     {
//       return std::make_unique<CalibrateFieldCoordinateAction>(name, "calibrate_field_coordinates", config);
//     };

//   factory.registerBuilder<CalibrateFieldCoordinateAction>("CalibrateFieldCoordinates", builder);
// }