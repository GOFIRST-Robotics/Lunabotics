// This node publishes CAN bus messages for our VESC burshless motor controllers.
// Original Author: Jude Sauve <sauve031@umn.edu> in 2018
// Maintainer: Anthony Brogni <brogn002@umn.edu>
// Last Updated: July 2023

// Import the ROS 2 Library
#include "rclcpp/rclcpp.hpp"

// Import ROS 2 Formatted Message Types
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

// Import Native C++ Libraries
#include <string>
#include <stdint.h>

// Global Variables //
float linear_drive_power_cmd = 0.0;
float angular_drive_power_cmd = 0.0;
float current_digger_RPM = 0.0;
bool digging = false;
bool reverse_digger = false;
bool offloading = false;

// Define CAN IDs Here //
const uint32_t FRONT_LEFT_DRIVE = 1;
const uint32_t BACK_LEFT_DRIVE = 4;
const uint32_t FRONT_RIGHT_DRIVE = 3;
const uint32_t BACK_RIGHT_DRIVE = 2;
const uint32_t DIGGER_ROTATION_MOTOR = 8;
const uint32_t DIGGER_DRUM_BELT_MOTOR = 7;
const uint32_t CONVEYOR_BELT_MOTOR = 6;
const uint32_t OFFLOAD_BELT_MOTOR = 5;

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

  // Set the speed of the motor in RPM (Rotations Per Minute)
  void vesc_set_RPM(uint32_t id, int rpm)
  {
    int32_t data = rpm;

    send_can(id + 0x00000300, data); // ID must be modified to signify this is an RPM command
    // RCLCPP_INFO(this->get_logger(), "Setting the RPM of CAN ID: %u to %d", id, rpm); // Print to the terminal
  }

  // Before sending CAN messages to the drivetrain motors, we want to desaturate the wheel speeds if needed
  void drive(float linear_power, float angular_power)
  {
    linear_power = std::clamp(linear_power, (float)(-1), (float)(1));   // Clamp the linear power between -1 and 1
    angular_power = std::clamp(angular_power, (float)(-1), (float)(1)); // Clamp the angular power between -1 and 1

    float leftPower = linear_power - angular_power;
    float rightPower = linear_power + angular_power;

    // Desaturate the wheel speeds if needed
    if (float greater_input = std::max(abs(leftPower), abs(rightPower)); greater_input > float(1))
    {
      float scale_factor = float(1) / greater_input;
      leftPower *= scale_factor;
      rightPower *= scale_factor;
    }

    vesc_set_duty_cycle(FRONT_LEFT_DRIVE, leftPower * -1);   // Multiply by -1 to invert motor direction
    vesc_set_duty_cycle(BACK_LEFT_DRIVE, leftPower * -1);    // Multiply by -1 to invert motor direction
    vesc_set_duty_cycle(FRONT_RIGHT_DRIVE, rightPower * -1); // Multiply by -1 to invert motor direction
    vesc_set_duty_cycle(BACK_RIGHT_DRIVE, rightPower * -1);  // Multiply by -1 to invert motor direction
  }

public:
  MotorControlNode() : Node("MotorControlNode")
  {
    // Define default values for our ROS parameters
    this->declare_parameter("DIGGER_ROTATION_POWER", 0.4); // Measured in duty cycle
    this->declare_parameter("DRUM_BELT_POWER", 0.2);       // Measured in duty cycle
    this->declare_parameter("CONVEYOR_BELT_POWER", 0.35);  // Measured in duty cycle
    this->declare_parameter("OFFLOAD_BELT_POWER", 0.35);   // Measured in duty cycle

    digger_RPM_pub = this->create_publisher<std_msgs::msg::Float32>("digger_RPM", 10);
    can_pub = this->create_publisher<can_msgs::msg::Frame>("CAN/slcan0/transmit", 100); // The name of this topic is determined by our CAN_bridge node

    can_sub = this->create_subscription<can_msgs::msg::Frame>("CAN/slcan0/receive", 10, std::bind(&MotorControlNode::CAN_callback, this, _1)); // The name of this topic is determined by our CAN_bridge node
    drive_power_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MotorControlNode::drive_power_callback, this, _1));
    actuators_sub = this->create_subscription<std_msgs::msg::String>("cmd_actuators", 10, std::bind(&MotorControlNode::actuators_callback, this, _1));

    timer = this->create_wall_timer(50ms, std::bind(&MotorControlNode::timer_callback, this));

    this->DIGGER_ROTATION_POWER = this->get_parameter("DIGGER_ROTATION_POWER").as_double();
    this->DRUM_BELT_POWER = this->get_parameter("DRUM_BELT_POWER").as_double();
    this->CONVEYOR_BELT_POWER = this->get_parameter("CONVEYOR_BELT_POWER").as_double();
    this->OFFLOAD_BELT_POWER = this->get_parameter("OFFLOAD_BELT_POWER").as_double();

    RCLCPP_INFO(this->get_logger(), "DIGGER_ROTATION_POWER has been set to: %f", DIGGER_ROTATION_POWER); // Print to the terminal
    RCLCPP_INFO(this->get_logger(), "DRUM_BELT_POWER has been set to: %f", DRUM_BELT_POWER);             // Print to the terminal
    RCLCPP_INFO(this->get_logger(), "CONVEYOR_BELT_POWER has been set to: %f", CONVEYOR_BELT_POWER);     // Print to the terminal
    RCLCPP_INFO(this->get_logger(), "OFFLOAD_BELT_POWER has been set to: %f", OFFLOAD_BELT_POWER);       // Print to the terminal
  }

  // Motor Speeds //
  float DIGGER_ROTATION_POWER;
  float DRUM_BELT_POWER;
  float CONVEYOR_BELT_POWER;
  float OFFLOAD_BELT_POWER;

private:
  void drive_power_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    linear_drive_power_cmd = msg->linear.x;
    angular_drive_power_cmd = msg->angular.z;
  }

  // Listen for status frames sent by our VESC motor controllers
  void CAN_callback(const can_msgs::msg::Frame::SharedPtr can_msg) const
  {
    uint32_t id = can_msg->id & 0xFF;

    uint32_t RPM = (can_msg->data[0] << 24) + (can_msg->data[1] << 16) + (can_msg->data[2] << 8) + can_msg->data[3];
    uint16_t dutyCycleNow = ((can_msg->data[6] << 8) + can_msg->data[7]) / 10;

    // RCLCPP_INFO(this->get_logger(), "Recieved status frame from CAN ID %u with the following data:", id);
    // RCLCPP_INFO(this->get_logger(), "RPM: %u Duty Cycle: %hu%%", RPM, dutyCycleNow);

    current_digger_RPM = RPM;
  }

  void actuators_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard this actuator_cmd: '%s'", msg->data.c_str());

    // Parse the msg for our toggleable motor actions
    if (msg->data.find("DIGGER_ON") != std::string::npos)
    {
      digging = true;
    }
    if (msg->data.find("OFFLOADER_ON") != std::string::npos)
    {
      offloading = true;
    }
    if (msg->data.find("DIGGER_OFF") != std::string::npos)
    {
      digging = false;
      reverse_digger = false;
    }
    if (msg->data.find("OFFLOADER_OFF") != std::string::npos)
    {
      offloading = false;
    }
    if (msg->data.find("REVERSE_DIGGER") != std::string::npos)
    {
      digging = false;
      reverse_digger = true;
    }
    if (msg->data.find("STOP_ALL_ACTUATORS") != std::string::npos)
    {
      digging = false;
      offloading = false;
      reverse_digger = false;
    }
  }

  // This method loops repeatedly
  void timer_callback()
  {
    // Drive the robot with the specified linear and angular speeds
    drive(linear_drive_power_cmd, angular_drive_power_cmd);

    // Send digging CAN messages
    if (digging)
    {
      vesc_set_duty_cycle(DIGGER_ROTATION_MOTOR, this->DIGGER_ROTATION_POWER * -1); // forwards
    }
    else if (reverse_digger)
    {
      vesc_set_duty_cycle(DIGGER_ROTATION_MOTOR, this->DIGGER_ROTATION_POWER); // backwards
    }
    else
    {
      vesc_set_duty_cycle(DIGGER_ROTATION_MOTOR, 0); // stop
    }
    vesc_set_duty_cycle(DIGGER_DRUM_BELT_MOTOR, digging ? this->DRUM_BELT_POWER * -1 : 0.0);
    vesc_set_duty_cycle(CONVEYOR_BELT_MOTOR, digging ? this->CONVEYOR_BELT_POWER : 0.0);

    // Send offloader CAN messages
    vesc_set_duty_cycle(OFFLOAD_BELT_MOTOR, offloading ? this->OFFLOAD_BELT_POWER * -1 : 0.0);

    // Publish the current digger speed in RPM to a topic
    std_msgs::msg::Float32 digger_RPM_msg;
    digger_RPM_msg.data = current_digger_RPM;
    digger_RPM_pub->publish(digger_RPM_msg);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr digger_RPM_pub;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_power_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr actuators_sub;
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
