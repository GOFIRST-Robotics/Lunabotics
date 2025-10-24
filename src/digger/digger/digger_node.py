# This ROS 2 node contains code for the digger subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: November 2023

import time

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Import ROS 2 formatted message types
from std_msgs.msg import Float32, Float32MultiArray

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet, MotorCommandGet
from rovr_interfaces.srv import SetPower, SetPosition
from rovr_interfaces.msg import Potentiometers
from std_srvs.srv import Trigger



#   void Potentiometer_callback(const rovr_interfaces::msg::Potentiometers msg) {
#     std_msgs::msg::Float32MultiArray digger_linear_actuator_msg;
#     digger_linear_actuator_msg.data = {this->can_data[this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int()].current, this->can_data[this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int()].current};
#     digger_linear_actuator_pub->publish(digger_linear_actuator_msg);

#     // // Linear actuators current spike detection
#     // double current_threshold = this->get_parameter("CURRENT_SPIKE_THRESHOLD").as_double(); // in amps
#     // double time_limit = this->get_parameter("CURRENT_SPIKE_TIME").as_double(); // in seconds
#     // double left_current = this->can_data[this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int()].current;
#     // double right_current = this->can_data[this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int()].current;
#     // // RCLCPP_INFO(this->get_logger(), "Left Current: %fA Right Current: %fA", left_current, right_current);
#     // if ((this->can_data[this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int()].dutyCycle < 0.0
#     // || this->can_data[this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int()].dutyCycle < 0.0)
#     // && (left_current > current_threshold || right_current > current_threshold)) {
#     //   if (start.has_value() && std::chrono::duration<double>(std::chrono::steady_clock::now() - *start).count() > time_limit) {
#     //       // Stop both linear actuators and the dumper motor
#     //       this->cli_lift_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
#     //       this->cli_digger_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
#     //       RCLCPP_WARN(this->get_logger(), "WARNING: Linear actuator current draw is too high! (%fA, %fA) Stopping the digger.", left_current, right_current);
#     //       return;
#     //   } else if (!start.has_value()) {
#     //       start = std::chrono::steady_clock::now();
#     //       RCLCPP_DEBUG(this->get_logger(), "Starting the timer for current spike detection.");
#     //   }
#     // } else if (start.has_value()) {
#     //     // Clear the start time when the current falls below the threshold
#     //     start.reset();
#     //     RCLCPP_DEBUG(this->get_logger(), "Resetting the timer for current spike detection.");
#     // }

#     // Digging buckets current spike detection
#     double buckets_current_threshold = this->get_parameter("BUCKETS_CURRENT_SPIKE_THRESHOLD").as_double(); // in amps
#     double buckets_time_limit = this->get_parameter("BUCKETS_CURRENT_SPIKE_TIME").as_double(); // in seconds
#     double buckets_current = this->can_data[this->get_parameter("DIGGER_MOTOR").as_int()].current;
#     // RCLCPP_INFO(this->get_logger(), "Digging Buckets Current: %fA", buckets_current);
#     if (buckets_current > buckets_current_threshold) {
#       if (buckets_start.has_value() && std::chrono::duration<double>(std::chrono::steady_clock::now() - *buckets_start).count() > buckets_time_limit) {
#           // Stop both linear actuators and the dumper motor
#           this->cli_lift_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
#           this->cli_digger_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
#           RCLCPP_WARN(this->get_logger(), "WARNING: Buckets current draw is too high! (%fA) Stopping the digger.", buckets_current);
#           return;
#       } else if (!buckets_start.has_value()) {
#           buckets_start = std::chrono::steady_clock::now();
#           RCLCPP_DEBUG(this->get_logger(), "Starting the timer for buckets current spike detection.");
#       }
#     } else if (buckets_start.has_value()) {
#         // Clear the start time when the current falls below the threshold
#         buckets_start.reset();
#         RCLCPP_DEBUG(this->get_logger(), "Resetting the timer for buckets current spike detection.");
#     }

#     float kP = static_cast<float>(this->get_parameter("DIGGER_ACTUATORS_kP").as_double());
#     int left_motor_pot = msg.left_motor_pot - this->get_parameter("DIGGER_ACTUATORS_OFFSET").as_int();
#     int right_motor_pot = msg.right_motor_pot;

#     double kP_coupling = this->get_parameter("DIGGER_ACTUATORS_kP_coupling").as_double();
#     int error = left_motor_pot - right_motor_pot;
#     float speed_adjustment_coupling = error * kP_coupling;

#     float kP_pitch = this->get_parameter("DIGGER_PITCH_kP").as_double(); 
#     float error_pitch = pitch - 0.0; // may need to adjust desired state from 0.0
#     float speed_adjustment_pitch = error_pitch * kP_pitch;

#     //RCLCPP_INFO(this->get_logger(), "Error: %d, Adjustment: %f", error, speed_adjustment);

#     if (abs(error) > this->get_parameter("MAX_POS_DIFF").as_int() && strcmp(this->digger_lift_goal.type.c_str(), "position") == 0) {
#       // Stop both motors!
#       this->cli_lift_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
#       // Log an error message
#       RCLCPP_ERROR(this->get_logger(), "ERROR: Position difference between linear actuators is too high! Stopping both motors.");
#     }
#     else if ((msg.left_motor_pot >= 1023) || (msg.right_motor_pot >= 1023)) {
#       // Stop both motors!
#       this->cli_lift_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
#       // Log an error message
#       RCLCPP_ERROR(this->get_logger(), "ERROR: Potentiometer has reached max value! Stopping both motors, check if one is unplugged");
#     }
#     else if ((msg.left_motor_pot <= 0) || (msg.right_motor_pot <= 0)) {
#       // Stop both motors!
#       this->cli_lift_stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
#       // Log an error message
#       RCLCPP_ERROR(this->get_logger(), "ERROR: Potentiometer has reached min value! Stopping both motors, check if one is unplugged");
#     }
#     else if (abs(error) > this->get_parameter("MAX_POS_DIFF").as_int() && strcmp(this->digger_lift_goal.type.c_str(), "duty_cycle") == 0 && this->digger_lift_goal.value != 0.0) {
#       RCLCPP_ERROR(this->get_logger(), "ERROR: Position difference between linear actuators is too high!");
#       if (error > 0.0 && this->digger_lift_goal.value > 0.0) {
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), 0.0);
#       } else if (error < 0.0 && this->digger_lift_goal.value > 0.0) {
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), 0.0);
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
#       } else if (error > 0.0 && this->digger_lift_goal.value < 0.0) {
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), 0.0);
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
#       } else if (error < 0.0 && this->digger_lift_goal.value < 0.0) {
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), 0.0);
#       }
#     }
#     else if (strcmp(this->digger_lift_goal.type.c_str(), "duty_cycle") == 0 && this->digger_lift_goal.value != 0.0) {
#       if (this->digger_lift_goal.value < 0.0 && this->get_parameter("TIPPING_SPEED_ADJUSTMENT").as_bool()) {
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value + speed_adjustment_coupling - speed_adjustment_pitch);
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value - speed_adjustment_coupling - speed_adjustment_pitch);
#       } else {
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value + speed_adjustment_coupling);
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value - speed_adjustment_coupling);
#       } 
#       //RCLCPP_INFO(this->get_logger(), "Output: %f, Pitch: %f", speed_adjustment_pitch, pitch);     
#     }
#     else if (strcmp(this->digger_lift_goal.type.c_str(), "duty_cycle") == 0) {
#       vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
#       vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), this->digger_lift_goal.value);
#     }
#     else if (strcmp(this->digger_lift_goal.type.c_str(), "position") == 0) {
#       int left_error = left_motor_pot - int(this->digger_lift_goal.value);
#       int right_error = right_motor_pot - int(this->digger_lift_goal.value);

#       float left_controller_output = std::clamp(kP * left_error, -this->digger_lift_goal.power_limit, this->digger_lift_goal.power_limit);
#       float right_controller_output = std::clamp(kP * right_error, -this->digger_lift_goal.power_limit, this->digger_lift_goal.power_limit);

#       if (left_error < 0 && right_error < 0 && this->get_parameter("TIPPING_SPEED_ADJUSTMENT").as_bool()) {
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), left_controller_output + speed_adjustment_coupling - speed_adjustment_pitch);
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), right_controller_output - speed_adjustment_coupling - speed_adjustment_pitch);
#       } else {
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").as_int(), left_controller_output + speed_adjustment_coupling);
#         vesc_set_duty_cycle(this->get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").as_int(), right_controller_output - speed_adjustment_coupling);
#       }

#       //RCLCPP_INFO(this->get_logger(), "Output: %f, Pitch: %f", speed_adjustment_pitch, pitch);       
#       //RCLCPP_INFO(this->get_logger(), "Current Pos: %d, Goal: %f, Output: %f", right_motor_pot, this->digger_lift_goal.value, right_controller_output);
#     } else{
#       RCLCPP_ERROR(this->get_logger(), "Unknown Digger Lift State: '%s'", this->digger_lift_goal.type.c_str());
#     }
#   }
class DiggerNode(Node):
    def __init__(self):
        """Initialize the ROS 2 digger node."""
        super().__init__("digger")

        self.cancel_current_srv = False
        self.long_service_running = False
        self.buckets_start = None
        

        # Calling the lift_stop service will cancel any long-running lift services!
        self.stop_lift_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")
        self.cli_digger_lift_set = self.create_client(MotorCommandSet, "digger_lift/set")
        self.cli_lift_stop = self.create_client(Trigger, "lift/stop")
        self.cli_digger_stop = self.create_client(Trigger, "digger/stop")

        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(
            SetPower, "digger/toggle", self.toggle_callback, callback_group=self.service_cb_group
        )
        self.srv_stop = self.create_service(
            Trigger, "digger/stop", self.stop_callback, callback_group=self.stop_lift_cb_group
        )
        self.srv_setPower = self.create_service(
            SetPower, "digger/setPower", self.set_power_callback, callback_group=self.service_cb_group
        )
        self.srv_setPosition = self.create_service(
            SetPosition, "lift/setPosition", self.set_position_callback, callback_group=self.service_cb_group
        )
        self.srv_lift_stop = self.create_service(
            Trigger, "lift/stop", self.stop_lift_callback, callback_group=self.stop_lift_cb_group
        )
        self.srv_lift_set_power = self.create_service(
            SetPower, "lift/setPower", self.lift_set_power_callback, callback_group=self.service_cb_group
        )
        self.srv_zero_lift = self.create_service(
            Trigger, "lift/zero", self.zero_lift_callback, callback_group=self.service_cb_group
        )
        self.srv_bottom_lift = self.create_service(
            Trigger, "lift/bottom", self.bottom_lift_callback, callback_group=self.service_cb_group
        )

        # Define subscribers here
        self.linear_actuator_duty_cycle_sub = self.create_subscription(
            Float32MultiArray, "Digger_Current", self.linear_actuator_current_callback, 10
        )
        self.potentiometer_sub = self.create_subscription(Potentiometers, "potentiometers", self.pot_callback, 10)

        # Define publishers here
        self.lift_pose_publisher = self.create_publisher(Float32, "lift_pose", 10)
        self.digger_linear_actuator_pub = self.create_publisher(Float32MultiArray, "Digger_Current", 5)

        # Define default values for our ROS parameters below #
        self.declare_parameter("digger_lift_manual_power_down", 0.12)
        self.declare_parameter("digger_lift_manual_power_up", 0.5)
        self.declare_parameter("DIGGER_MOTOR", 3)
        self.declare_parameter("DIGGER_ACTUATORS_OFFSET", 12)
        self.declare_parameter("DIGGER_SAFETY_ZONE", 120)  # Measured in potentiometer units (0 to 1023)
        self.declare_parameter("DIGGER_LEFT_LINEAR_ACTUATOR", 8) 
        self.declare_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR", 7)
        self.declare_parameter("BUCKETS_CURRENT_SPIKE_THRESHOLD", 8.0) # In amps
        self.declare_parameter("BUCKETS_CURRENT_SPIKE_TIME", 0.2) # In seconds
        self.declare_parameter("DIGGER_MOTOR", 10)
        self.declare_parameter("DIGGER_ACTUATORS_kP", 0.05)
        self.declare_parameter("DIGGER_ACTUATORS_OFFSET", 12)
        self.declare_parameter("DIGGER_ACTUATORS_kP_coupling", 0.10)
        self.declare_parameter("DIGGER_PITCH_kP", 2.5)
        self.declare_parameter("MAX_POS_DIF", 30)
        self.declare_parameter("TIPPING_SPEED_ADJUSTMENT", False)
        # Assign the ROS Parameters to member variables below #
        self.digger_lift_manual_power_down = self.get_parameter("digger_lift_manual_power_down").value
        self.digger_lift_manual_power_up = self.get_parameter("digger_lift_manual_power_up").value
        self.DIGGER_MOTOR = self.get_parameter("DIGGER_MOTOR").value
        self.DIGGER_ACTUATORS_OFFSET = self.get_parameter("DIGGER_ACTUATORS_OFFSET").value
        self.DIGGER_SAFETY_ZONE = self.get_parameter("DIGGER_SAFETY_ZONE").value
        # Print the ROS Parameters to the terminal below #
        self.get_logger().info(
            "digger_lift_manual_power_down has been set to: " + str(self.digger_lift_manual_power_down)
        )
        self.get_logger().info("digger_lift_manual_power_up has been set to: " + str(self.digger_lift_manual_power_up))
        self.get_logger().info("DIGGER_MOTOR has been set to: " + str(self.DIGGER_MOTOR))
        self.get_logger().info("DIGGER_ACTUATORS_OFFSET has been set to: " + str(self.DIGGER_ACTUATORS_OFFSET))
        self.get_logger().info("DIGGER_SAFETY_ZONE has been set to: " + str(self.DIGGER_SAFETY_ZONE))
        # Current state of the digger chain
        self.running = False
        # Current position of the lift motor in potentiometer units (0 to 1023)
        self.current_lift_position = None  # We don't know the current position yet
        # Goal Threshold
        self.goal_threshold = 2  # in potentiometer units (0 to 1023)
        # Current state of the lift system
        self.lift_lowering = False

        # Linear Actuator Current Threshold
        self.current_threshold = 0.3
        self.left_linear_actuator_current = 0.0
        self.right_linear_actuator_current = 0.0


    
    def potentiometer_callback(self, msg: Potentiometers) -> None:
        digger_linear_actuator_msg = [self.can_data[self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value].current, self.can_data[self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value].current]
        self.digger_linear_actuator_pub.publish(digger_linear_actuator_msg)

        # Digging buckets current spike detection
        buckets_current_threshold = self.get_parameter("BUCKETS_CURRENT_SPIKE_THRESHOLD").value
        buckets_time_limit = self.get_parameter("BUCKETS_CURRENT_SPIKE_TIME").value
        buckets_current = self.can_data[self.get_parameter("DIGGER_MOTOR").value].current

        if buckets_current > buckets_current_threshold:
            if self.buckets_start is not None and (time.monotonic() - self.buckets_start) > buckets_time_limit:
                # Stop both linear actuators and the dumper motor
                self.cli_lift_stop.call_async(Trigger.Request)
                self.cli_digger_stop.call_async(Trigger.Request)
                rclpy.get_logger().warn("WARNING: Buckets current draw is too high! " + buckets_current + "Stopping the digger.")
                return
            elif self.buckets_start is None:
                self.buckets_start = time.monotonic()
                rclpy.get_logger().debug(self.get_logger(), "Starting the timer for buckets current spike detection.")
        elif self.buckets_start is not None:
            # Clear the start time when the current falls below the threshold
            self.buckets_start = None
            rclpy.get_logger().debug(self.get_logger(), "Resetting the timer for buckets current spike detection.")

        kP = float(self.get_parameter("DIGGER_ACTUATORS_kP").value)
        left_motor_pot = msg.left_motor_pot - self.get_parameter("DIGGER_ACTUATORS_OFFSET").value
        right_motor_pot = msg.right_motor_pot

        kP_coupling = self.get_parameter("DIGGER_ACTUATORS_kP_coupling").value 
        error = left_motor_pot - right_motor_pot
        speed_adjustment_coupling = error * kP_coupling

        kP_pitch = self.get_parameter("DIGGER_PITCH_kP").value
        error_pitch = self.pitch - 0.0 # may need to adjust desired state from 0.0
        speed_adjustment_pitch = error_pitch * kP_pitch

        if abs(error) > self.get_parameter("MAX_POS_DIF").value and self.digger_lift_goal.type == "position":
            # Stop both motors!
            self.cli_lift_stop.call_async(Trigger.Request)
            # Log an error message
            rclpy.get_logger().error(self.get_logger(), "ERROR: Position difference between linear actuators is too high! Stopping both motors.")
        elif msg.left_motor_pot >= 1023 or msg.right_motor_pot >= 1023:
            # Stop both motors!
            self.cli_lift_stop.call_async(Trigger.Request)
            # Log an error message
            rclpy.get_logger().error(self.get_logger(), "ERROR: Potentiometer has reached max value! Stopping both motors, check if one is unplugged")
        elif msg.left_motor_pot <= 0 or msg.right_motor_pot <= 0:
            # Stop both motors!
            self.cli_lift_stop.call_async(Trigger.Request)
            # Log an error message
            rclpy.get_logger().error(self.get_logger(), "ERROR: Potentiometer has reached min value! Stopping both motors, check if one is unplugged")
        elif abs(error) > self.get_parameter("MAX_POS_DIF").value and self.digger_lift_goal_type.type == "duty_cycle" and self.digger_lift_goal != 0.0:
            rclpy.get_logger().error(self.get_logger(), "ERROR: Position difference between linear actuators is too high!")
            if error > 0.0 and self.digger_lift_goal > 0.0:
                self.set_power(self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value, self.digger_lift_goal)
                self.set_power(self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value, 0.0)
            elif error < 0.0 and self.digger_lift_goal > 0.0:
                self.set_power(self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value, 0.0)
                self.set_power(self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value, self.digger_lift_goal)
            elif error > 0.0 and self.digger_lift_goal < 0.0:
                self.set_power(self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value, 0.0)
                self.set_power(self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value, self.digger_lift_goal)
            elif error < 0.0 and self.digger_lift_goal < 0.0:
                self.set_power(self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value, self.digger_lift_goal)
                self.set_power(self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value, 0.0)
        elif self.digger_lift_goal == "duty_cycle" and self.digger_lift_goal != 0.0:
            if self.digger_lift_goal < 0.0 and self.get_parameter("TIPPING_SPEED_ADJUSTMENT").value:
               self.set_power(self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value, self.digger_lift_goal + speed_adjustment_coupling - speed_adjustment_pitch)
               self.set_power(self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value, self.digger_lift_goal - speed_adjustment_coupling - speed_adjustment_pitch) 
            else:
               self.set_power(self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value, self.digger_lift_goal + speed_adjustment_coupling)
               self.set_power(self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value, self.digger_lift_goal - speed_adjustment_coupling)
        elif self.digger_lift_goal == "duty_cycle":
            self.set_power(self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value, self.digger_lift_goal)
            self.set_power(self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value, self.digger_lift_goal)
        elif self.digger_lift_goal == "position":
            left_error = left_motor_pot - int(self.digger_lift_goal)
            right_error = right_motor_pot - int(self.digger_lift_goal)

            left_controller_output = max(-self.digger_lift_goal.power_limit, min(kP * left_error, self.digger_lift_goal.power_limit))

            right_controller_output = max(-self.digger_lift_goal.power_limit, min(kP * right_error, self.digger_lift_goal.power_limit))
   
            if (left_error < 0 and right_error < 0 and self.get_parameter("TIPPING_SPEED_ADJUSTMENT").value):
                self.set_power(self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value, left_controller_output + speed_adjustment_coupling - speed_adjustment_pitch)
                self.set_power(self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value, right_controller_output - speed_adjustment_coupling - speed_adjustment_pitch)
            else:
                self.set_power(self.get_parameter("DIGGER_LEFT_LINEAR_ACTUATOR").value, left_controller_output + speed_adjustment_coupling)
                self.set_power(self.get_parameter("DIGGER_RIGHT_LINEAR_ACTUATOR").value, right_controller_output - speed_adjustment_coupling)
        else:
            rclpy.get_logger().error(self.logger(), "Unknown Digger Lift State: '%s'", self.digger_lift_goal.type)
    
    pitch = 0.0

    # Define subsystem methods here
    def set_power(self, digger_power: float) -> None:
        """This method sets power to the digger chain."""
        if self.current_lift_position < self.DIGGER_SAFETY_ZONE - 5:
            self.get_logger().warn("WARNING: The digger is not extended enough! Stopping the buckets.")
            self.stop()  # Stop the digger chain
        else:
            self.running = True
            self.cli_motor_set.call_async(
                MotorCommandSet.Request(type="duty_cycle", can_id=self.DIGGER_MOTOR, value=digger_power)
            )

    def stop(self) -> None:
        """This method stops the digger chain."""
        self.running = False
        self.cli_motor_set.call_async(MotorCommandSet.Request(type="duty_cycle", can_id=self.DIGGER_MOTOR, value=0.0))

    def toggle(self, digger_chain_power: float) -> None:
        """This method toggles the digger chain."""
        if self.running:
            self.stop()
        else:
            self.set_power(digger_chain_power)

    def set_position(self, position: int, power_limit: float = 0.5) -> bool:
        """This method sets the position of the digger lift and waits until the goal is reached."""
        self.lift_lowering = position > self.current_lift_position
        if self.lift_lowering and (not self.running) and (self.current_lift_position >= self.DIGGER_SAFETY_ZONE):
            self.get_logger().warn(
                "WARNING: The digger buckets are not running! Will not lower.", throttle_duration_sec=5
            )
            self.stop_lift()  # Stop the lift system
            return False
        self.get_logger().info("Setting the lift position to: " + str(position))
        self.long_service_running = True
        self.cli_digger_lift_set.call_async(
            MotorCommandSet.Request(
                type="position",
                value=float(position),
                power_limit=power_limit,
            )
        )
        # Wait until the goal position goal is reached to return
        while abs(position - self.current_lift_position) > self.goal_threshold:
            if self.cancel_current_srv:
                break
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
        self.long_service_running = False
        if self.cancel_current_srv:
            self.cancel_current_srv = False
            self.get_logger().info("Lift position goal was stopped.")
            return False
        self.get_logger().info("Done setting the lift position to: " + str(position))
        return True

    def stop_lift(self) -> None:
        """This method stops the lift."""
        self.cli_digger_lift_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                value=0.0,
            )
        )

    def lift_set_power(self, power: float) -> None:
        """This method sets power to the lift system."""
        self.lift_lowering = power < 0
        if self.lift_lowering and (not self.running) and (self.current_lift_position >= self.DIGGER_SAFETY_ZONE):
            self.get_logger().warn(
                "WARNING: The digger buckets are not running! Will not lower.", throttle_duration_sec=5
            )
            self.stop_lift()  # Stop the lift system
            return
        self.cli_digger_lift_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                value=power,
            )
        )

    def zero_lift(self) -> None:
        """This method zeros the lift system by slowly raising it until the duty cycle is 0."""
        self.get_logger().info("Zeroing the lift system")
        self.long_service_running = True
        self.lift_set_power(self.digger_lift_manual_power_up)
        lastPowerTime = time.time()
        # Wait 0.5 seconds after the current goes below the threshold before stopping the motor
        while time.time() - lastPowerTime < 0.5:
            if self.cancel_current_srv:
                self.cancel_current_srv = False
                break
            # If the current is not below the threshold, update the last power time
            if not (
                self.left_linear_actuator_current < self.current_threshold
                or self.right_linear_actuator_current < self.current_threshold
            ):
                lastPowerTime = time.time()
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
            # self.get_logger().info("time.time() - lastPowerTime is currently: " + str(time.time() - lastPowerTime))
        self.stop_lift()
        self.long_service_running = False
        self.get_logger().info("Done zeroing the lift system")

    def bottom_lift(self) -> None:
        """This method bottoms out the lift system by slowly lowering it until the duty cycle is 0."""
        self.get_logger().info("Bottoming out the lift system")
        self.long_service_running = True
        self.lift_set_power(-self.digger_lift_manual_power_down)
        lastPowerTime = time.time()
        # Wait 0.5 seconds after the current goes below the threshold before stopping the motor
        while time.time() - lastPowerTime < 0.5:
            if self.cancel_current_srv:
                self.cancel_current_srv = False
                break
            # If the current is not below the threshold, update the last power time
            if not (
                self.left_linear_actuator_current < self.current_threshold
                or self.right_linear_actuator_current < self.current_threshold
            ):
                lastPowerTime = time.time()
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
            # self.get_logger().info("time.time() - lastPowerTime is currently: " + str(time.time() - lastPowerTime))
        self.stop_lift()
        self.long_service_running = False
        self.get_logger().info("Done bottoming out the lift system")

    # Define service callback methods here
    def set_power_callback(self, request, response):
        """This service request sets power to the digger chain."""
        self.set_power(request.power)
        response.success = True
        return response

    def stop_callback(self, request, response):
        """This service request stops the digger chain."""
        self.stop()
        response.success = True
        return response

    def toggle_callback(self, request, response):
        """This service request toggles the digger chain."""
        self.toggle(request.power)
        response.success = True
        return response

    def set_position_callback(self, request, response):
        """This service request sets the position of the lift."""
        response.success = self.set_position(request.position, request.power_limit)
        return response

    def stop_lift_callback(self, request, response):
        """This service request stops the lift system."""
        if self.long_service_running:
            self.cancel_current_srv = True
        self.stop_lift()
        response.success = True
        return response

    def lift_set_power_callback(self, request, response):
        """This service request sets power to the digger chain."""
        self.lift_set_power(request.power)
        response.success = True
        return response

    def zero_lift_callback(self, request, response):
        """This service request zeros the lift system."""
        self.zero_lift()
        response.success = True
        return response

    def bottom_lift_callback(self, request, response):
        """This service request bottoms out the lift system."""
        self.bottom_lift()
        response.success = True
        return response

    # Define the subscriber callback for the potentiometers topic
    def pot_callback(self, msg: Potentiometers):
        """Helps us know whether or not the current goal position has been reached."""
        # Average the two potentiometer values
        self.current_lift_position = ((msg.left_motor_pot - self.DIGGER_ACTUATORS_OFFSET) + msg.right_motor_pot) / 2
        self.lift_pose_publisher.publish(Float32(data=self.current_lift_position))
        if self.current_lift_position < self.DIGGER_SAFETY_ZONE and self.running:
            self.get_logger().warn("WARNING: The digger is not extended enough! Stopping the buckets.")
            self.stop()  # Stop the digger chain
        if self.lift_lowering and (not self.running) and (self.current_lift_position >= self.DIGGER_SAFETY_ZONE):
            self.get_logger().warn(
                "WARNING: The digger buckets are not running! Will not lower.", throttle_duration_sec=5
            )
            self.stop_lift()  # Stop the lift system

    # Define subscriber callback methods here
    def linear_actuator_current_callback(self, linear_acutator_msg):
        self.left_linear_actuator_current = linear_acutator_msg.data[0]
        self.right_linear_actuator_current = linear_acutator_msg.data[1]


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = DiggerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    node.get_logger().info("Initializing the Digger subsystem!")
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
