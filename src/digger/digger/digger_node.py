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
from std_msgs.msg import Float32MultiArray

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet, MotorCommandGet
from rovr_interfaces.srv import SetPower, SetPosition
from rovr_interfaces.msg import Potentiometers
from std_srvs.srv import Trigger


class DiggerNode(Node):
    def __init__(self):
        """Initialize the ROS 2 digger node."""
        super().__init__("digger")

        self.cancel_current_srv = False
        self.long_service_running = False

        # Calling the lift_stop service will cancel any long-running lift services!
        self.stop_lift_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")
        self.cli_digger_lift_set = self.create_client(MotorCommandSet, "digger_lift/set")

        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(
            SetPower, "digger/toggle", self.toggle_callback, callback_group=self.service_cb_group
        )
        self.srv_stop = self.create_service(
            Trigger, "digger/stop", self.stop_callback, callback_group=self.service_cb_group
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

        # Define default values for our ROS parameters below #
        self.declare_parameter("DIGGER_MOTOR", 3)
        self.declare_parameter("DIGGER_ACTUATORS_OFFSET", 12)
        self.declare_parameter("DIGGER_SAFETY_ZONE", 75)  # Measured in potentiometer units (0 to 1023) # TODO: Tune
        # Assign the ROS Parameters to member variables below #
        self.DIGGER_MOTOR = self.get_parameter("DIGGER_MOTOR").value
        self.DIGGER_ACTUATORS_OFFSET = self.get_parameter("DIGGER_ACTUATORS_OFFSET").value
        self.DIGGER_SAFETY_ZONE = self.get_parameter("DIGGER_SAFETY_ZONE").value
        # Print the ROS Parameters to the terminal below #
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
        self.lift_running = False

        # Linear Actuator Current Threshold
        self.current_threshold = 0.3
        self.left_linear_actuator_current = 0.0
        self.right_linear_actuator_current = 0.0

    # Define subsystem methods here
    def set_power(self, digger_power: float) -> None:
        """This method sets power to the digger chain."""
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

    def set_position(self, position: int) -> None:
        """This method sets the position of the digger lift and waits until the goal is reached."""
        if position > self.current_lift_position and not self.running and self.current_lift_position >= self.DIGGER_SAFETY_ZONE:
            self.get_logger().warn("WARNING: The digger buckets are not running! Will not lower.")
            self.stop_lift()  # Stop the lift system
            return
        self.get_logger().info("Setting the lift position to: " + str(position))
        self.long_service_running = True
        self.cli_digger_lift_set.call_async(
            MotorCommandSet.Request(
                type="position",
                value=float(position),
            )
        )
        # Wait until the goal position goal is reached to return
        while abs(position - self.current_lift_position) > self.goal_threshold:
            if self.cancel_current_srv:
                self.cancel_current_srv = False
                break
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
        self.long_service_running = False
        self.get_logger().info("Done setting the lift position to: " + str(position))

    def stop_lift(self) -> None:
        """This method stops the lift."""
        self.lift_running = False
        self.cli_digger_lift_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                value=0.0,
            )
        )

    def lift_set_power(self, power: float) -> None:
        """This method sets power to the lift system."""
        self.lift_running = True
        if power < 0 and not self.running and self.current_lift_position >= self.DIGGER_SAFETY_ZONE:
            self.get_logger().warn("WARNING: The digger buckets are not running! Will not lower.")
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
        self.lift_set_power(0.05)
        while not (
            self.left_linear_actuator_current < self.current_threshold
            or self.right_linear_actuator_current < self.current_threshold
        ):
            if self.cancel_current_srv:
                self.cancel_current_srv = False
                break
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
        self.stop_lift()
        self.long_service_running = False
        self.get_logger().info("Done zeroing the lift system")

    def bottom_lift(self) -> None:
        """This method bottoms out the lift system by slowly lowering it until the duty cycle is 0."""
        self.get_logger().info("Bottoming out the lift system")
        self.long_service_running = True
        self.lift_set_power(-0.05)
        while not (
            self.left_linear_actuator_current < self.current_threshold
            or self.right_linear_actuator_current < self.current_threshold
        ):
            if self.cancel_current_srv:
                self.cancel_current_srv = False
                break
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
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
        self.set_position(request.position)
        response.success = True
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
        if self.current_lift_position < self.DIGGER_SAFETY_ZONE and self.running:
            self.get_logger().warn("WARNING: The digger is not extended enough! Stopping the buckets.")
            self.stop()  # Stop the digger chain

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
