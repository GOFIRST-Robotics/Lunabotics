// This node publishes CAN bus messages to our VESC brushless motor controllers.
// Original Author: Jude Sauve <sauve031@umn.edu> in 2018
// Maintainer: Anthony Brogni <brogn002@umn.edu>
// Last Updated: September 2023

// Import the ROS 2 Library
#include "rclcpp/rclcpp.hpp"

// Import ROS 2 Formatted Message Types
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rovr_interfaces/msg/motor_command.hpp"

// Import Native C++ Libraries
#include <string>
#include <stdint.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class MotorControlNode : public rclcpp::Node
{
  // Generic method for sending data over the CAN bus
  void send_can(uint32_t id, int32_t data) const
  {
    can_msgs::msg::Frame can_msg; // Construct a new CAN message

    can_msg.is_rtr = false;
    can_msg.is_error = false;
    can_msg.is_extended = true;

    can_msg.id = id; // Set the CAN ID for this message

    can_msg.dlc = 4;                       // Size of the data array (how many bytes to use, maximum of 8)
    can_msg.data[0] = (data >> 24) & 0xFF; // First byte of data
    can_msg.data[1] = (data >> 16) & 0xFF; // Second byte of data
    can_msg.data[2] = (data >> 8) & 0xFF;  // Third byte of data
    can_msg.data[3] = data & 0xFF;         // Fourth byte of data

    can_pub->publish(can_msg); // Publish our new CAN message to the ROS 2 topic
  }

  // Set the percent power of the motor between -1.0 and 1.0
  void vesc_set_duty_cycle(uint32_t id, float percentPower)
  {
    percentPower = std::clamp(percentPower, (float)(-1), (float)(1)); // Do not allow setting more than 100% power in either direction

    int32_t data = percentPower * 100000; // Convert from percent power to a signed 32-bit integer

    send_can(id + 0x00000000, data); // ID does NOT need to be modified to signify this is a duty cycle command
    // RCLCPP_INFO(this->get_logger(), "Setting the duty cycle of CAN ID: %u to %f", id, percentPower); // Print to the terminal
  }

  // Set the velocity of the motor in RPM (Rotations Per Minute)
  void vesc_set_velocity(uint32_t id, int rpm)
  {
    int32_t data = rpm;

    send_can(id + 0x00000300, data); // ID must be modified to signify this is an RPM command
    // RCLCPP_INFO(this->get_logger(), "Setting the RPM of CAN ID: %u to %d", id, rpm); // Print to the terminal
  }

  // Set the position of the motor in _____ (degrees? encoder counts?)
  void vesc_set_position(uint32_t id, int position)
  {
    // TODO: Implement this method!
  }

  // Set the current draw of the motor in amps
  void vesc_set_current(uint32_t id, float current)
  {
    // TODO: Implement this method!
  }


public:
  MotorControlNode() : Node("MotorControlNode")
  {
    // Define default values for our ROS parameters
    this->declare_parameter("DIGGER_ROTATION_POWER", 0.4); // Measured in duty cycle
    this->declare_parameter("DRUM_BELT_POWER", 0.2);       // Measured in duty cycle
    this->declare_parameter("CONVEYOR_BELT_POWER", 0.35);  // Measured in duty cycle
    this->declare_parameter("OFFLOAD_BELT_POWER", 0.35);   // Measured in duty cycle

    can_pub = this->create_publisher<can_msgs::msg::Frame>("CAN/slcan0/transmit", 100); // The name of this topic is determined by ros2socketcan_bridge
    can_sub = this->create_subscription<can_msgs::msg::Frame>("CAN/slcan0/receive", 10, std::bind(&MotorControlNode::CAN_callback, this, _1)); // The name of this topic is determined by ros2socketcan_bridge
    command_sub = this->create_subscription<rovr_interfaces::msg::MotorCommand>("motor_cmd", 10, std::bind(&MotorControlNode::command_callback, this, _1)); // The name of this topic is determined by ros2socketcan_bridge
  }

private:
  // Listen for CAN status frames sent by our VESC motor controllers
  void CAN_callback(const can_msgs::msg::Frame::SharedPtr can_msg)
  {
    uint32_t id = can_msg->id & 0xFF;

    uint32_t RPM = (can_msg->data[0] << 24) + (can_msg->data[1] << 16) + (can_msg->data[2] << 8) + can_msg->data[3];
    uint16_t dutyCycleNow = ((can_msg->data[6] << 8) + can_msg->data[7]) / 10;
    // TODO: calculate the current draw of the motor from the CAN data
    // TODO: calculate the position of the motor from the CAN data

    // RCLCPP_INFO(this->get_logger(), "Recieved status frame from CAN ID %u with the following data:", id);
    // RCLCPP_INFO(this->get_logger(), "RPM: %u Duty Cycle: %hu%%", RPM, dutyCycleNow);
  }

  // Execute the specified motor command
  void command_callback(const rovr_interfaces::msg::MotorCommand::SharedPtr command)
  {
    if(command->type == "velocity") {
      vesc_set_velocity(command->can_id, command->value);
    } else if (command->type == "duty_cycle") {
      vesc_set_duty_cycle(command->can_id, command->value);
    } else if (command->type == "position") {
      vesc_set_position(command->can_id, command->value);
    } else if (command->type == "current") {
      vesc_set_current(command->can_id, command->value);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown motor command type: '%s'", command->type);
    }
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub;
  rclcpp::Subscription<rovr_interfaces::msg::MotorCommand>::SharedPtr command_sub;
};

// Main method for the node
int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Spin the node
  rclcpp::spin(std::make_shared<MotorControlNode>());

  // Free up any resources being used by the node
  rclcpp::shutdown();
  return 0;
}
