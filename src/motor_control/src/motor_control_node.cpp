/*
 * motor_control_node.cpp
 * Sends raw canbus msgs according to motor config.
 * VERSION: 0.0.5
 * Last changed: February 2023
 * Original Author: Jude Sauve <sauve031@umn.edu>
 * Maintainer: Anthony Brogni <brogn002@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// Import ROS 2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

// Import Native C++ Libraries
#include <string>
#include <stdint.h>

// Global Variables
float linear_drive_power_cmd = 0.0;
float angular_drive_power_cmd = 0.0;
bool digging = false;
bool offloading = false;

// Define CAN IDs Here //
uint32_t FRONT_LEFT_DRIVE = 1;
uint32_t BACK_LEFT_DRIVE = 2;
uint32_t FRONT_RIGHT_DRIVE = 3;
uint32_t BACK_RIGHT_DRIVE = 4;
uint32_t DIGGER_DEPTH_MOTOR = 5;
uint32_t DIGGER_ROTATION_MOTOR = 6;
uint32_t DIGGER_DRUM_BELT_MOTOR = 7;
uint32_t CONVEYOR_BELT_MOTOR = 8;
uint32_t OFFLOAD_BELT_MOTOR = 9;

// Define Motor Power/Speeds Here //
float DIGGER_ROTATION_POWER = 0.5;
float DIGGER_DEPTH_POWER = 0.5;
float DRUM_BELT_POWER = 0.5;
float CONVEYOR_BELT_POWER = 0.5;
float OFFLOAD_BELT_POWER = 0.5;

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublishersAndSubscribers : public rclcpp::Node
{
  // Generic method for sending data over the CAN bus
  void send_can(uint32_t id, int32_t data) {
    can_msgs::msg::Frame can_msg; // Construct a new CAN message

    can_msg.is_rtr = false;
    can_msg.is_error = false;
    can_msg.is_extended = true;

    can_msg.id = id; // Set the CAN ID for this message

    can_msg.dlc = 4; // Size of the data array (how many bytes to use, maximum of 8)
    can_msg.data[0] = (data >> 24) & 0xFF; // First byte of data
    can_msg.data[1] = (data >> 16) & 0xFF; // Second byte of data
    can_msg.data[2] = (data >> 8) & 0xFF; // Third byte of data
    can_msg.data[3] = data & 0xFF; // Fourth byte of data
    
    can_pub->publish(can_msg); // Publish our new CAN message to the ROS 2 topic
  }

  // Set the percent power of the motor between -1.0 and 1.0
  void vesc_set_duty_cycle(uint32_t id, float percentPower) { 
    int32_t data = percentPower * 100000; // Convert from percent power to a signed 32-bit integer

    send_can(id + 0x00000000, data); // ID does NOT need to be modified to signify this is a duty cycle command
    RCLCPP_INFO(this->get_logger(), "Setting the duty cycle of CAN ID: %u to %f", id, percentPower); // Print to the terminal
  }

  // Set the current draw of the motor in amps
  void vesc_set_current(uint32_t id, float current) { 
    int32_t data = current * 1000; // Convert from current in amps to a signed 32-bit integer

    send_can(id + 0x00000100, data); // ID must be modified to signify this is a current command
    RCLCPP_INFO(this->get_logger(), "Setting the current draw of CAN ID: %u to %f amps", id, current); // Print to the terminal
  }

  // TODO: This has not been tested yet
  // Set the speed of the motor in RPM (Rotations Per Minute)
  void vesc_set_RPM(uint32_t id, int rpm) {
    int32_t data = rpm;

    send_can(id + 0x00000300, data); // ID must be modified to signify this is an RPM command
    RCLCPP_INFO(this->get_logger(), "Setting the RPM of CAN ID: %u to %d", id, rpm); // Print to the terminal
  }

  // TODO: This has not been tested yet
  // Set the position of the motor in encoder counts
  void vesc_set_position(uint32_t id, int encoderCounts) {
    int32_t data = encoderCounts;

    send_can(id + 0x00000400, data); // ID must be modified to signify this is a position command
    RCLCPP_INFO(this->get_logger(), "Setting the position of CAN ID: %u to %d", id, encoderCounts); // Print to the terminal
  }

public:
  PublishersAndSubscribers()
  : Node("publishers_and_subscribers")
  {
    can_pub = this->create_publisher<can_msgs::msg::Frame>("CAN/can0/transmit", 100); // The name of this topic is determined by our CAN_bridge node
    can_sub = this->create_subscription<can_msgs::msg::Frame>("CAN/can1/receive", 10, std::bind(&PublishersAndSubscribers::CAN_callback, this, _1)); // The name of this topic is determined by our CAN_bridge node
    drive_power_sub = this->create_subscription<geometry_msgs::msg::Twist>("drive_power", 10, std::bind(&PublishersAndSubscribers::drive_power_callback, this, _1));
    actuators_sub = this->create_subscription<std_msgs::msg::String>("cmd_actuators", 10, std::bind(&PublishersAndSubscribers::actuators_callback, this, _1));
    timer = this->create_wall_timer(50ms, std::bind(&PublishersAndSubscribers::timer_callback, this));
  }

private:
  void drive_power_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    linear_drive_power_cmd = msg->linear.x;
    angular_drive_power_cmd = msg->angular.z;
  }

  // TODO: The values we are receiving/print still seem to be wrong?? (Besides the CAN ID, that has been working)
  // Listen for status frames sent by our VESC motor controllers
  void CAN_callback(const can_msgs::msg::Frame::SharedPtr can_msg) const
  {
    uint32_t id = can_msg->id & 0xFF;

    uint32_t RPM = (can_msg->data[0]<<24) + (can_msg->data[1]<<16) + (can_msg->data[2]<<8) + can_msg->data[3];
    uint16_t avgMotorCurrent =((can_msg->data[4]<<8) + can_msg->data[5]) / 10;
    uint16_t dutyCycleNow = ((can_msg->data[6]<<8) + can_msg->data[7]) / 1000;

    RCLCPP_INFO(this->get_logger(), "Recieved status frame from CAN ID %u with the following data:", id);
    RCLCPP_INFO(this->get_logger(), "RPM: %u average motor current: %hu latest duty cycle: %hu", RPM, avgMotorCurrent, dutyCycleNow);
  }

  void actuators_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard this actuator_cmd: '%s'", msg->data.c_str());
    
    // Parse the msg for our toggleable motor actions
    if (msg->data.find("STOP_ALL_ACTUATORS") != std::string::npos) {
      digging = false;
      offloading = false;
    }
    if(msg->data.find("DIGGER_ON") != std::string::npos) {
      digging = true;
    }
    if(msg->data.find("OFFLOADER_ON") != std::string::npos) {
      offloading = true;
    }
    if(msg->data.find("DIGGER_OFF") != std::string::npos) {
      digging = false;
    }
    if(msg->data.find("OFFLOADER_OFF") != std::string::npos) {
      offloading = false;
    }

    // Parse the msg for our linear actuator position commands
    if(msg->data.find("EXTEND_DIGGER") != std::string::npos) {
      // TODO: Set position of the linear actuator by sending a 1 over UART to the Arduino
    }
    if(msg->data.find("RETRACT_DIGGER") != std::string::npos) {
      // TODO: Set position of the linear actuator by sending a 0 over UART to the Arduino
    }

    if(msg->data.find("BEGIN_DIG_PROCEDURE") != std::string::npos) { //TODO: fix this
      // TODO: Wait until our digger is up to speed
      // TODO: Set position of the linear actuator to lower the digger
      // TODO: After the digger is fully lowered into the ground, begin slowly driving
    }
  }

  // This method loops repeatedly
  void timer_callback()
  {
    // Send drivetrain CAN messages
    vesc_set_duty_cycle(FRONT_LEFT_DRIVE, linear_drive_power_cmd - angular_drive_power_cmd);
    vesc_set_duty_cycle(BACK_LEFT_DRIVE, linear_drive_power_cmd - angular_drive_power_cmd);
    vesc_set_duty_cycle(FRONT_RIGHT_DRIVE, (linear_drive_power_cmd + angular_drive_power_cmd) * -1); // Multiply by -1 to invert motor direction
    vesc_set_duty_cycle(BACK_RIGHT_DRIVE, (linear_drive_power_cmd + angular_drive_power_cmd) * -1); // Multiply by -1 to invert motor direction

    // Send digging CAN messages
    vesc_set_duty_cycle(DIGGER_ROTATION_MOTOR, digging ? DIGGER_ROTATION_POWER : 0.0);
    vesc_set_duty_cycle(DIGGER_DRUM_BELT_MOTOR, digging ? DRUM_BELT_POWER : 0.0);
    vesc_set_duty_cycle(CONVEYOR_BELT_MOTOR, digging ? CONVEYOR_BELT_POWER : 0.0);

    // Send offloader CAN messages
    vesc_set_duty_cycle(OFFLOAD_BELT_MOTOR, offloading ? CONVEYOR_BELT_POWER : 0.0);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_power_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr actuators_sub;
};

// Main method for the node
int main(int argc, char** argv){
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Spin the node
  rclcpp::spin(std::make_shared<PublishersAndSubscribers>());

  // Free up any resources being used by the node
  rclcpp::shutdown();
  return 0;
}