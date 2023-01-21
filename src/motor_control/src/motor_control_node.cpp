/*
 * motor_control_node.cpp
 * Publishes raw CAN messages to the "CAN/can0/transmit" ROS2 topic.
 * VERSION: 0.0.4
 * Last changed: January 2023
 * Original Author: Jude Sauve <sauve031@umn.edu>
 * Maintainer: Anthony Brogni <brogn002@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// Import ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

// Import Native C++ Libraries
#include <string>
#include <stdint.h>
typedef int32_t S32;
typedef uint32_t U32;

// ROS2 Parameters // TODO: Not setup as parameters yet
double linear_scale = 1.0;
double angular_scale = 1.0;
double digger_scale = 1.0;
bool digging = false;

// Global Variables
void send_can(U32 id, S32 data);
void send_can_bool(U32 id, bool data);

// Declare CAN IDs Here //
int front_left_drive = (U32) 1;
int front_right_drive = (U32) 2;
int back_left_drive = (U32) 3;
int back_right_drive = (U32) 4;
int digger_motor = (U32) 5;

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublishersAndSubscribers : public rclcpp::Node
{
  // Method for sending data over the CAN bus
  void send_can(U32 id, S32 data){
    can_msgs::msg::Frame can_msg;
    can_msg.is_rtr = false;
    can_msg.is_extended = true;
    can_msg.is_error = false;
    can_msg.dlc = 4U;
    can_msg.id = id;
    can_msg.data[3] = data & 0xFF;
    can_msg.data[2] = (data >> 8) & 0xFF;
    can_msg.data[1] = (data >> 16) & 0xFF;
    can_msg.data[0] = (data >> 24) & 0xFF;
    can_pub->publish(can_msg); // Publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing a CAN message to CAN ID: %i", can_msg.id); // Print to the terminal
  }

  // Method for sending a boolean value over the CAN bus
  void send_can_bool(U32 id, bool data){
    can_msgs::msg::Frame can_msg;
    can_msg.is_rtr = false;
    can_msg.is_extended = true;
    can_msg.is_error = false;
    can_msg.dlc = 1U;
    can_msg.id = id;
    can_msg.data[0] = data;
    can_pub->publish(can_msg); // Publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing a boolean CAN message to CAN ID: %i", can_msg.id); // Print to the terminal
  }

public:
  PublishersAndSubscribers()
  : Node("publishers_and_subscribers")
  {
    can_pub = this->create_publisher<can_msgs::msg::Frame>("CAN/can0/transmit", 1); // The name of this topic is determined by our CAN_bridge node
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&PublishersAndSubscribers::velocity_callback, this, _1));
    actuators_sub = this->create_subscription<std_msgs::msg::String>("cmd_actuators", 1, std::bind(&PublishersAndSubscribers::actuators_callback, this, _1));
  }

private:
  void actuators_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard this actuator_cmd: '%s'", msg->data.c_str());
    
    // Determine whether the digging should be on or off right now
    if(msg->data == "DIGGER_ON") {
        digging = true;
    } else if(msg->data == "DiGGER_OFF") {
        digging = false;
    }
  }

  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    // Send drivetrain CAN messages
    send_can(front_left_drive, (linear_vel_cmd*linear_scale - angular_vel_cmd*angular_scale) * 100000);
    send_can(back_left_drive, (linear_vel_cmd*linear_scale - angular_vel_cmd*angular_scale) * 100000);
    send_can(front_right_drive, (linear_vel_cmd*linear_scale + angular_vel_cmd*angular_scale) * -100000); // Add negative sign to invert the motor
    send_can(back_right_drive, (linear_vel_cmd*linear_scale + angular_vel_cmd*angular_scale) * -100000); // Add negative sign to invert the motor

    // Send digger CAN message
    send_can(digger_motor, digging ? digger_scale * 100000.0 : 0.0);
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr actuators_sub;
};

// Main method for the node
int main(int argc, char** argv){
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Spin the node
  rclcpp::spin(std::make_shared<PublishersAndSubscribers>());

  // Free up any resources being used by the node
  rclcpp::shutdown();
  return 0;
}