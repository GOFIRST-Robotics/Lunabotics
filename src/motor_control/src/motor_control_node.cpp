// This node publishes CAN bus messages to our VESC brushless motor controllers.
// Original Author: Jude Sauve <sauve031@umn.edu> in 2018
// Maintainer: Anthony Brogni <brogn002@umn.edu>
// Last Updated: November 2023

// Import the ROS 2 Library
#include "rclcpp/rclcpp.hpp"

// Import ROS 2 Formatted Message Types
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_srvs/srv/trigger.hpp"

// Import custom ROS 2 interfaces
#include "rovr_interfaces/srv/motor_command_get.hpp"
#include "rovr_interfaces/srv/motor_command_set.hpp"
#include "rovr_interfaces/msg/potentiometers.hpp"

// Import Native C++ Libraries
#include <chrono>
#include <map>
#include <memory>
#include <stdint.h>
#include <string>
#include <tuple> // for tuples
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// Calculates the modulus of an input value within a given range.
// If the input is above the maximum input, it wraps around to the minimum input.
// If the input is below the minimum input, it wraps around to the maximum input.
double inputModulus(double input, double minimumInput, double maximumInput) {
  double modulus = maximumInput - minimumInput;

  // Wrap input if it's above the maximum input
  int numMax = (int) ((input - minimumInput) / modulus);
  input -= numMax * modulus;

  // Wrap input if it's below the minimum input
  int numMin = (int) ((input - maximumInput) / modulus);
  input -= numMin * modulus;

  return input;
}

// Define a struct to store motor data
struct MotorData {
  float dutyCycle;
  float velocity;
  float current;
  int tachometer;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;
};

// Define a struct to store the current digger lift goal
struct DiggerLiftGoal {
  std::string type;
  float value;
};

class PIDController {
private:
  int COUNTS_PER_REVOLUTION; // How many encoder counts for one 360 degree rotation
  int DEAD_BAND; // How close to the target position is close enough
  float MAX_POWER; // Cap the power output to the motor (this should be between 0 and 1)

  bool continuous; // Does the input range wrap around (e.g. absolute encoder)
  int minimumTachInput, maximumTachInput; // For continuous input, what is the range?

  float kp, ki, kd;
  float gravComp; // Gravity compensation constant (if needed)

  int32_t targTach, totalError;

  std::optional<int32_t> prevError;

public:
  bool isActive;

  PIDController(int CountsPerRevolution, float kp, float ki, float kd, float gravComp, int deadband, float maxPower) {
    this->COUNTS_PER_REVOLUTION = CountsPerRevolution;
    this->DEAD_BAND = deadband;
    this->MAX_POWER = maxPower;

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->gravComp = gravComp;
    this->continuous = false;

    this->totalError = 0;
    this->isActive = false;
    this->prevError = std::nullopt;
  }

  float update(int32_t currTach) {
    // Calculate the current error
    float currError;
    if (this->continuous) {
      double errorBound = (this->maximumTachInput - this->minimumTachInput) / 2.0;
      currError = inputModulus(this->targTach - currTach, -errorBound, errorBound);
    } else {
      currError = (this->targTach - currTach);
    }

    // If the error is within the dead band, return early
    if (abs(currError) <= DEAD_BAND) {
      return 0;
    }
  
    this->totalError += currError;
  
    float PIDResult = (currError * this->kp) + (this->totalError * this->ki) + (this->prevError.has_value() ? currError - this->prevError.value() : 0) * this->kd;

    PIDResult = std::clamp(PIDResult, (float)(-this->MAX_POWER), (float)(this->MAX_POWER)); // Clamp the PIDResult to the maximum power

    // Uncomment the line below for debug values:
    // std::cout << "Target Tachometer: " << targTach << ", Current Tachometer: " << currTach << ", Current Error: " << currError << ", PIDResult: " << PIDResult << ", Total Error: " << totalError << ", D: " << (this->prevError.has_value() ? currError - this->prevError.value() : 0) << std::endl;
    
    this->prevError = currError; // Assign the previous error to the current error

    return PIDResult;
  }

  void setRotation(float degrees) {
    this->isActive = true;
    this->targTach = static_cast<int32_t>((degrees / 360.0) * this->COUNTS_PER_REVOLUTION);
    this->totalError = 0;
    this->prevError = std::nullopt;
  }

  int getCountsPerRevolution() {
    return this->COUNTS_PER_REVOLUTION;
  }

  // Methods for continuous input
  void enableContinuousInput(int minDegrees, int maxDegrees) {
    this->continuous = true;
    this->minimumTachInput = static_cast<int32_t>((minDegrees / 360.0) * this->COUNTS_PER_REVOLUTION);
    this->maximumTachInput = static_cast<int32_t>((maxDegrees / 360.0) * this->COUNTS_PER_REVOLUTION);
  }
  void disableContinuousInput() {
    this->continuous = false;
  }
  bool isContinuousInputEnabled() {
    return this->continuous;
  }
};

class MotorControlNode : public rclcpp::Node {
  // Generic method for sending data over the CAN bus
  void send_can(uint32_t id, int32_t data) const {
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

    can_pub->publish(can_msg); // Publish the CAN message to the ROS 2 topic
  }

  // Set the percent power of the motor between -1.0 and 1.0
  void vesc_set_duty_cycle(uint32_t id, float percentPower) {
    // Disable the PID controller for this motor if it's active
    if (this->pid_controllers[id]) {
      this->pid_controllers[id]->isActive = false;
    }

    // Do not allow setting more than 100% power in either direction
    percentPower = std::clamp(percentPower, (float)(-1), (float)(1));
    int32_t data = percentPower * 100000; // Convert from percent power to a signed 32-bit integer

    send_can(id + 0x00000000, data); // ID does NOT need to be modified to signify this is a duty cycle command
    this->current_msg[id] = std::make_tuple(id + 0x00000000, data); // update the hashmap
    RCLCPP_DEBUG(this->get_logger(), "Setting the duty cycle of CAN ID: %u to %f", id, percentPower); // Print Statement
  }

  // Smoothly ramp the motor speed to the dutyCycleGoal over <time> seconds
  void vesc_ramp_duty_cycle(uint32_t id, float dutyCycleGoal, float time) {
    const auto start = std::chrono::steady_clock::now(); // time at the start of the service
    std::chrono::duration<double> elapsedTime = start - start; // time elapsed since the start of the service
    std::optional<float> initialDutyCycleOption = vesc_get_duty_cycle(id); // initial duty cycle of the motor
    float initialDutyCycle = 0.0;
    if (initialDutyCycleOption.has_value()) {
      initialDutyCycle = initialDutyCycleOption.value();
    } else {
      RCLCPP_ERROR(this->get_logger(), "ERROR: initialDutyCycle is NULL! Aborting vesc_ramp_duty_cycle.");
      return;
    }
    auto lastDebugTime = start; // time of the last debug statement
    
    // Update the duty cycle by (std::min(elapsedTime.count() / time, 1.0) * (dutyCycleGoal - initialDutyCycle)) 
    // each iteration until the time elapsed is greater than or equal to the time parameter.
    while (elapsedTime.count() < time) {
      const auto currentTime = std::chrono::steady_clock::now();
      elapsedTime = currentTime - start;
      // This is a linear interpolation between the initial duty cycle and the goal duty cycle
      float progress = (std::min(elapsedTime.count() / time, 1.0)); // progress is a value between 0 and 1
      float newDutyCycle = initialDutyCycle + (progress * (dutyCycleGoal - initialDutyCycle));
      vesc_set_duty_cycle(id, newDutyCycle);

      // Print debug statements every 100ms
      if ((currentTime - lastDebugTime) >= 100ms) {
        RCLCPP_DEBUG(this->get_logger(), "Current duty cycle set: %f", newDutyCycle);
        RCLCPP_DEBUG(this->get_logger(), "Time elapsed: %f seconds", elapsedTime.count());
        lastDebugTime = currentTime;
      }
    }

    // make sure the duty cycle ends at exactly the goal
    vesc_set_duty_cycle(id, dutyCycleGoal);
  }

  // Set the velocity of the motor in RPM (Rotations Per Minute)
  void vesc_set_velocity(uint32_t id, int rpm) {
    if (this->pid_controllers[id]) {
      this->pid_controllers[id]->isActive = false;
    }
    int32_t data = rpm;

    send_can(id + 0x00000300, data); // ID must be modified to signify this is a RPM command
    this->current_msg[id] = std::make_tuple(id + 0x00000300, data); // update the hashmap
    RCLCPP_DEBUG(this->get_logger(), "Setting the velocity of CAN ID: %u to %d RPM", id, rpm); // Print Statement
  }

  // Set the position of the motor in degrees
  void vesc_set_position(uint32_t id, int position) {
    if (this->pid_controllers[id]) {
      this->pid_controllers[id]->setRotation(position);
    }
    RCLCPP_DEBUG(this->get_logger(), "Setting the position of CAN ID: %u to %d degrees", id, position); // Print Statement
  }

  void set_digger_lift_callback(const std::shared_ptr<rovr_interfaces::srv::MotorCommandSet::Request> request,
                    std::shared_ptr<rovr_interfaces::srv::MotorCommandSet::Response> response) {
    // Update the current digger lift goal
    this->digger_lift_goal = { request->type, request->value };
    response->success = true;
  }

  // Get the motor controller's current duty cycle command
  std::optional<float> vesc_get_duty_cycle(uint32_t id) {
    if (std::chrono::steady_clock::now() - this->can_data[id].timestamp < this->threshold) {
      return this->can_data[id].dutyCycle;
    } else {
      return std::nullopt; // The data is too stale
    }
  }
  // Get the current velocity of the motor in RPM (Rotations Per Minute)
  std::optional<float> vesc_get_velocity(uint32_t id) {
    if (std::chrono::steady_clock::now() - this->can_data[id].timestamp < this->threshold) {
      return this->can_data[id].velocity;
    } else {
      return std::nullopt; // The data is too stale
    }
  }
  // Get the current position (tachometer reading) of the motor
  std::optional<float> vesc_get_position(uint32_t id) {
    if (std::chrono::steady_clock::now() - this->can_data[id].timestamp < this->threshold) {
      return (static_cast<float>(this->can_data[id].tachometer) / static_cast<float>(this->pid_controllers[id]->getCountsPerRevolution())) * 360.0;
    } else {
      return std::nullopt; // The data is too stale
    }
  }
   // Get the current draw of the motor in amps
  std::optional<float> vesc_get_current(uint32_t id) {
    if (std::chrono::steady_clock::now() - this->can_data[id].timestamp < this->threshold) {
      return this->can_data[id].current;
    } else {
      return std::nullopt; // The data is too stale
    }
  }

public:
  MotorControlNode() : Node("MotorControlNode") {
    // Define default values for our ROS parameters below #
    this->declare_parameter("CAN_INTERFACE_TRANSMIT", "can0");
    this->declare_parameter("CAN_INTERFACE_RECEIVE", "can0");
    this->declare_parameter("DIGGER_LEFT_LINEAR_ACTUATOR", 2);
    this->declare_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR", 1);
    this->declare_parameter("MAX_POS_DIFF", 30);
    this->declare_parameter("DUMPER_MOTOR", 24);
    this->declare_parameter("DIGGER_ACTUATORS_OFFSET", 12);
    this->declare_parameter("DIGGER_ACTUATORS_kP", 0.05);
    this->declare_parameter("DIGGER_ACTUATORS_kP_coupling", 0.10);
    this->declare_parameter("DIGGER_PITCH_kP", 2.5);
    this->declare_parameter("TIPPING_SPEED_ADJUSTMENT", true);
    this->declare_parameter("CURRENT_SPIKE_THRESHOLD", 1.8); // TODO: Tune this on the real robot!
    this->declare_parameter("CURRENT_SPIKE_TIME", 0.2); // TODO: Tune this on the real robot!

    // Print the ROS Parameters to the terminal below #
    RCLCPP_INFO(this->get_logger(), "CAN_INTERFACE_TRANSMIT parameter set to: %s", this->get_parameter("CAN_INTERFACE_TRANSMIT").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "CAN_INTERFACE_RECEIVE parameter set to: %s", this->get_parameter("CAN_INTERFACE_RECEIVE").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "DIGGER_LEFT_LINEAR_ACTUATOR parameter set to: %ld", this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int());
    RCLCPP_INFO(this->get_logger(), "DIGGER_RIGHT_LINEAR_ACTUATOR parameter set to: %ld", this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int());
    RCLCPP_INFO(this->get_logger(), "MAX_POS_DIFF parameter set to: %ld", this->get_parameter("MAX_POS_DIFF").as_int());
    RCLCPP_INFO(this->get_logger(), "DUMPER_MOTOR parameter set to: %ld", this->get_parameter("DUMPER_MOTOR").as_int());
    RCLCPP_INFO(this->get_logger(), "DIGGER_ACTUATORS_OFFSET parameter set to: %ld", this->get_parameter("DIGGER_ACTUATORS_OFFSET").as_int());
    RCLCPP_INFO(this->get_logger(), "DIGGER_ACTUATORS_kP parameter set to: %f", this->get_parameter("DIGGER_ACTUATORS_kP").as_double());
    RCLCPP_INFO(this->get_logger(), "DIGGER_ACTUATORS_kP_coupling parameter set to: %f", this->get_parameter("DIGGER_ACTUATORS_kP_coupling").as_double());
    RCLCPP_INFO(this->get_logger(), "DIGGER_PITCH_kP parameter set to: %f", this->get_parameter("DIGGER_PITCH_kP").as_double());
    RCLCPP_INFO(this->get_logger(), "TIPPING_SPEED_ADJUSTMENT parameter set to: %d", this->get_parameter("TIPPING_SPEED_ADJUSTMENT").as_bool());
    RCLCPP_INFO(this->get_logger(), "CURRENT_SPIKE_THRESHOLD parameter set to: %f", this->get_parameter("CURRENT_SPIKE_THRESHOLD").as_double());
    RCLCPP_INFO(this->get_logger(), "CURRENT_SPIKE_TIME parameter set to: %f", this->get_parameter("CURRENT_SPIKE_TIME").as_double());

    // Initialize services below //
    srv_motor_set = this->create_service<rovr_interfaces::srv::MotorCommandSet>(
        "motor/set", std::bind(&MotorControlNode::set_callback, this, _1, _2));
    srv_motor_get = this->create_service<rovr_interfaces::srv::MotorCommandGet>(
        "motor/get", std::bind(&MotorControlNode::get_callback, this, _1, _2));
    srv_set_digger_lift = this->create_service<rovr_interfaces::srv::MotorCommandSet>(
        "digger_lift/set", std::bind(&MotorControlNode::set_digger_lift_callback, this, _1, _2));

    // Initialize timers below //
    timer = this->create_wall_timer(500ms, std::bind(&MotorControlNode::timer_callback, this));

    cli_digger_stop = this->create_client<std_srvs::srv::Trigger>("digger/stop");

    digger_linear_actuator_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("Digger_Current", 5);
    dumper_linear_actuator_pub = this->create_publisher<std_msgs::msg::Float32>("Dumper_Current", 5);
    // Initialize publishers and subscribers below //
    // The name of this topic is determined by ros2socketcan_bridge
    can_pub = this->create_publisher<can_msgs::msg::Frame>("CAN/" + this->get_parameter("CAN_INTERFACE_TRANSMIT").as_string() + "/transmit", 100);
    // The name of this topic is determined by ros2socketcan_bridge
    can_sub = this->create_subscription<can_msgs::msg::Frame>("CAN/" + this->get_parameter("CAN_INTERFACE_RECEIVE").as_string() + "/receive", 10, std::bind(&MotorControlNode::CAN_callback, this, _1));

    potentiometer_sub = this->create_subscription<rovr_interfaces::msg::Potentiometers>("potentiometers", 10, std::bind(&MotorControlNode::Potentiometer_callback, this, _1));
    pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/zed2i/zed_node/pose", 10, std::bind(&MotorControlNode::Pose_callback, this, _1));

    // Initialize the current digger lift goal
    this->digger_lift_goal = { "duty_cycle", 0.0 }; // Stopped by default
  }

private:
  // Continuously send CAN messages to our motor controllers so they don't timeout
  void timer_callback() {
    // Loop through everything in the current_msg hashmap
    for (auto pair : this->current_msg) {
      uint32_t motorId = pair.first;
      // If the motor controller has previously received a command, send the most recent command again
      if ((this->pid_controllers[motorId] && this->pid_controllers[motorId]->isActive == false) || this->pid_controllers[motorId] == NULL) {
        send_can(std::get<0>(this->current_msg[motorId]), std::get<1>(this->current_msg[motorId]));
      }
    }
  }

  // Listen for CAN status frames sent by our VESC motor controllers
  void CAN_callback(const can_msgs::msg::Frame::SharedPtr can_msg) {
    uint32_t motorId = can_msg->id & 0xFF;
    uint32_t statusId = (can_msg->id >> 8) & 0xFF; // Packet Status, not frame ID

    // If 'motorId' is not found in the 'can_data' hashmap, add it.
    if (this->can_data.count(motorId) == 0) {
      this->can_data[motorId] = {0, 0, 0, 0, std::chrono::steady_clock::now()};
    }

    float dutyCycleNow = this->can_data[motorId].dutyCycle;
    float RPM = this->can_data[motorId].velocity;
    float current = this->can_data[motorId].current;
    int32_t tachometer = this->can_data[motorId].tachometer;
    std_msgs::msg::Float32 dumper_linear_actuator_msg;

    switch (statusId) {
    case 9: // Packet Status 9 (RPM & Duty Cycle)
      RPM = static_cast<float>((can_msg->data[0] << 24) + (can_msg->data[1] << 16) + (can_msg->data[2] << 8) + can_msg->data[3]);
      current = static_cast<float>(static_cast<short>((can_msg->data[4] << 8) + can_msg->data[5])) / 10.0; 
      dutyCycleNow = static_cast<float>(static_cast<short>((can_msg->data[6] << 8) + can_msg->data[7])) / 10.0 / 100.0;
      dumper_linear_actuator_msg.data = this->can_data[this->get_parameter("DUMPER_MOTOR").as_int()].current;
      dumper_linear_actuator_pub->publish(dumper_linear_actuator_msg);
      break;
    case 27: // Packet Status 27 (Tachometer)
      tachometer = static_cast<int32_t>((can_msg->data[0] << 24) + (can_msg->data[1] << 16) + (can_msg->data[2] << 8) + can_msg->data[3]);
      
      // Runs the PID controller for this motor if its active
      if (this->pid_controllers[motorId] && this->pid_controllers[motorId]->isActive) {
        float PIDResult = this->pid_controllers[motorId]->update(tachometer);

        int32_t data = PIDResult * 100000; // Convert from percent power to a signed 32-bit integer
        send_can(motorId + 0x00000000, data); // ID does NOT need to be modified to signify this is a duty cycle command
      }

      break;
    }

    // Store the most recent motor data in the hashmap
    this->can_data[motorId] = {dutyCycleNow, RPM, current, tachometer, std::chrono::steady_clock::now()};

    RCLCPP_DEBUG(this->get_logger(), "Received status frame %u from CAN ID %u with the following data:", statusId, motorId);
    RCLCPP_DEBUG(this->get_logger(), "RPM: %.2f Duty Cycle: %.2f%% Current: %.2fAmps Tachometer: %d", RPM, dutyCycleNow, current, tachometer);
  }

  void Potentiometer_callback(const rovr_interfaces::msg::Potentiometers msg) {
    std_msgs::msg::Float32MultiArray digger_linear_actuator_msg;
    digger_linear_actuator_msg.data = {this->can_data[this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int()].current, this->can_data[this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int()].current};
    digger_linear_actuator_pub->publish(digger_linear_actuator_msg);

    double current_threshold = this->get_parameter("CURRENT_SPIKE_THRESHOLD").as_double(); // in amps
    double time_limit = this->get_parameter("CURRENT_SPIKE_TIME").as_double(); // in seconds
    double left_current = this->can_data[this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int()].current;
    double right_current = this->can_data[this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int()].current;
    RCLCPP_INFO(this->get_logger(), "Left Current: %fA Right Current: %fA", left_current, right_current);
    if ((this->can_data[this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int()].dutyCycle < 0.0
    || this->can_data[this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int()].dutyCycle < 0.0)
    && (left_current > current_threshold || right_current > current_threshold)) {
      if (start.has_value() && std::chrono::duration<double>(std::chrono::steady_clock::now() - *start).count() > time_limit) {
          this->digger_lift_goal = { "duty_cycle", 0.0 };  
          vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), 0.0);
          vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), 0.0);
          this->cli_digger_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
          RCLCPP_WARN(this->get_logger(), "WARNING: Linear actuator current draw is too high! (%fA, %fA) Stopping the digger.", left_current, right_current);
          return;
      } else if (!start.has_value()) {
          start = std::chrono::steady_clock::now();
          RCLCPP_DEBUG(this->get_logger(), "Starting the timer for current spike detection.");
      }
    } else if (start.has_value()) {
        // Clear the start time when the current falls below the threshold
        start.reset();
        RCLCPP_DEBUG(this->get_logger(), "Resetting the timer for current spike detection.");
    }

    double kP = this->get_parameter("DIGGER_ACTUATORS_kP").as_double();
    int left_motor_pot = msg.left_motor_pot - this->get_parameter("DIGGER_ACTUATORS_OFFSET").as_int();
    int right_motor_pot = msg.right_motor_pot;

    double kP_coupling = this->get_parameter("DIGGER_ACTUATORS_kP_coupling").as_double();
    int error = left_motor_pot - right_motor_pot;
    float speed_adjustment_coupling = error * kP_coupling;

    float kP_pitch = this->get_parameter("DIGGER_PITCH_kP").as_double(); 
    float error_pitch = pitch - 0.0; // may need to adjust desired state from 0.0
    float speed_adjustment_pitch = error_pitch * kP_pitch;

    //RCLCPP_INFO(this->get_logger(), "Error: %d, Adjustment: %f", error, speed_adjustment);

    if (abs(error) > this->get_parameter("MAX_POS_DIFF").as_int() && strcmp(this->digger_lift_goal.type.c_str(), "position") == 0) {
      // Stop both motors!
      this->digger_lift_goal = { "duty_cycle", 0.0 };
      vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), 0.0);
      vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), 0.0);
      // Log an error message
      RCLCPP_ERROR(this->get_logger(), "ERROR: Position difference between linear actuators is too high! Stopping both motors.");
    }
    else if ((msg.left_motor_pot == 1023) || (msg.right_motor_pot == 1023)) {
      // Stop both motors!
      this->digger_lift_goal = { "duty_cycle", 0.0 };
      vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), 0.0);
      vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), 0.0);
      // Log an error message
      RCLCPP_ERROR(this->get_logger(), "ERROR: Potentiometer has reached max value! Stopping both motors, check if one is unplugged");
    }
    else if ((msg.left_motor_pot == 0) || (msg.right_motor_pot == 0)) {
      // Stop both motors!
      this->digger_lift_goal = { "duty_cycle", 0.0 };
      vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), 0.0);
      vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), 0.0);
      // Log an error message
      RCLCPP_ERROR(this->get_logger(), "ERROR: Potentiometer has reached min value! Stopping both motors, check if one is unplugged");
    }
    else if (abs(error) > this->get_parameter("MAX_POS_DIFF").as_int() && strcmp(this->digger_lift_goal.type.c_str(), "duty_cycle") == 0 && this->digger_lift_goal.value != 0.0) {
      RCLCPP_ERROR(this->get_logger(), "ERROR: Position difference between linear actuators is too high!");
      if (error > 0.0 && this->digger_lift_goal.value > 0.0) {
        vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
        vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), 0.0);
      } else if (error < 0.0 && this->digger_lift_goal.value > 0.0) {
        vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), 0.0);
        vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
      } else if (error > 0.0 && this->digger_lift_goal.value < 0.0) {
        vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), 0.0);
        vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
      } else if (error < 0.0 && this->digger_lift_goal.value < 0.0) {
        vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
        vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), 0.0);
      }
    }
    else if (strcmp(this->digger_lift_goal.type.c_str(), "duty_cycle") == 0 && this->digger_lift_goal.value != 0.0) {
      if (this->digger_lift_goal.value < 0.0 && this->get_parameter("TIPPING_SPEED_ADJUSTMENT").as_bool()) {
        vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value + speed_adjustment_coupling - speed_adjustment_pitch);
        vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value - speed_adjustment_coupling - speed_adjustment_pitch);
      } else {
        vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value + speed_adjustment_coupling);
        vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value - speed_adjustment_coupling);
      } 
      //RCLCPP_INFO(this->get_logger(), "Output: %f, Pitch: %f", speed_adjustment_pitch, pitch);     
    }
    else if (strcmp(this->digger_lift_goal.type.c_str(), "duty_cycle") == 0) {
      vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
      vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
    }
    else if (strcmp(this->digger_lift_goal.type.c_str(), "position") == 0) {
      int left_error = left_motor_pot - int(this->digger_lift_goal.value);
      int right_error = right_motor_pot - int(this->digger_lift_goal.value);

      double left_controller_output = std::clamp(kP * left_error, -0.5, 0.5);
      double right_controller_output = std::clamp(kP * right_error, -0.5, 0.5);
     
      if (left_error < 0 && right_error < 0 && this->get_parameter("TIPPING_SPEED_ADJUSTMENT").as_bool()) {
        vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), left_controller_output + speed_adjustment_coupling - speed_adjustment_pitch);
        vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), right_controller_output - speed_adjustment_coupling - speed_adjustment_pitch);
      } else {
        vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), left_controller_output + speed_adjustment_coupling);
        vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), right_controller_output - speed_adjustment_coupling);
      } 

      //RCLCPP_INFO(this->get_logger(), "Output: %f, Pitch: %f", speed_adjustment_pitch, pitch);       
      //RCLCPP_INFO(this->get_logger(), "Current Pos: %d, Goal: %f, Output: %f", right_motor_pot, this->digger_lift_goal.value, right_controller_output);
    } else{
      RCLCPP_ERROR(this->get_logger(), "Unknown Digger Lift State: '%s'", this->digger_lift_goal.type.c_str());
    }
  }

  void Pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, yaw;
      m.getRPY(roll, pitch, yaw);
  }

  // Initialize a hashmap to store the most recent motor data for each CAN ID
  std::map<uint32_t, MotorData> can_data;
  std::map<uint32_t, PIDController*> pid_controllers;

  double pitch = 0.0;
  std::optional<std::chrono::steady_clock::time_point> start;
  DiggerLiftGoal digger_lift_goal;

  // Adjust this data retention threshold as needed
  const std::chrono::seconds threshold = std::chrono::seconds(1);

  // Initialize a hashmap to store the most recent msg for each CAN ID
  std::map<uint32_t, std::tuple<uint32_t, int32_t>> current_msg;

  // Callback method for the MotorCommandSet service
  void set_callback(const std::shared_ptr<rovr_interfaces::srv::MotorCommandSet::Request> request,
                    std::shared_ptr<rovr_interfaces::srv::MotorCommandSet::Response> response) {

    if (strcmp(request->type.c_str(), "velocity") == 0) {
      vesc_set_velocity(request->can_id, request->value);
      response->success = true;
    } else if (strcmp(request->type.c_str(), "duty_cycle") == 0) {
      vesc_set_duty_cycle(request->can_id, request->value);
      response->success = true;
    } else if (strcmp(request->type.c_str(), "position") == 0) {
      vesc_set_position(request->can_id, request->value);
      response->success = true;
    } else if (strcmp(request->type.c_str(), "ramp_duty_cycle") == 0){
      vesc_ramp_duty_cycle(request->can_id, request->value, request->value2);
      response->success = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown motor SET command type: '%s'", request->type.c_str());
      response->success = false;
    }
  }

  // Callback method for the MotorCommandGet service
  void get_callback(const std::shared_ptr<rovr_interfaces::srv::MotorCommandGet::Request> request,
                    std::shared_ptr<rovr_interfaces::srv::MotorCommandGet::Response> response) {
    std::optional<float> data = std::nullopt;

    if (strcmp(request->type.c_str(), "velocity") == 0) {
      data = vesc_get_velocity(request->can_id);
    } else if (strcmp(request->type.c_str(), "duty_cycle") == 0) {
      data = vesc_get_duty_cycle(request->can_id);
    } else if (strcmp(request->type.c_str(), "position") == 0) {
      data = vesc_get_position(request->can_id);
    } else if (strcmp(request->type.c_str(), "current") == 0) {
      data = vesc_get_current(request->can_id);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown motor GET command type: '%s'", request->type.c_str());
    }

    if (data.has_value()) {
      response->data = data.value();
      response->success = true;
    } else {
      response->success = false;
      RCLCPP_WARN(this->get_logger(), "GET command for CAN ID %u read stale data!", request->can_id);
    }
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_digger_stop;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr digger_linear_actuator_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dumper_linear_actuator_pub;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub;
  rclcpp::Subscription<rovr_interfaces::msg::Potentiometers>::SharedPtr potentiometer_sub; 
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  rclcpp::Service<rovr_interfaces::srv::MotorCommandSet>::SharedPtr srv_motor_set;
  rclcpp::Service<rovr_interfaces::srv::MotorCommandGet>::SharedPtr srv_motor_get;
  rclcpp::Service<rovr_interfaces::srv::MotorCommandSet>::SharedPtr srv_set_digger_lift;
};

// Main method for the node
int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Spin the node
  rclcpp::spin(std::make_shared<MotorControlNode>());

  // Free up any resources being used by the node
  rclcpp::shutdown();
  return 0;
}
