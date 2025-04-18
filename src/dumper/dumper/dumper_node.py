# This ROS 2 node contains code for the dumper subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2024
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2024

# Import the ROS 2 Python module
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet, MotorCommandGet, SetPower
from std_srvs.srv import Trigger
from std_msgs.msg import Float32


class DumperNode(Node):
    def __init__(self):
        """Initialize the ROS 2 dumper node."""
        super().__init__("dumper")

        self.cancel_current_srv = False
        self.long_service_running = False

        # Calling the stop service will cancel any long-running services!
        self.stop_service_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(
            Trigger, "dumper/toggle", self.toggle_callback, callback_group=self.service_cb_group
        )
        self.srv_stop = self.create_service(
            Trigger, "dumper/stop", self.stop_callback, callback_group=self.stop_service_cb_group
        )
        self.srv_setPower = self.create_service(
            SetPower, "dumper/setPower", self.set_power_callback, callback_group=self.service_cb_group
        )

        self.srv_extendDumper = self.create_service(
            Trigger, "dumper/extendDumper", self.extend_callback, callback_group=self.service_cb_group
        )
        self.srv_retractDumper = self.create_service(
            Trigger, "dumper/retractDumper", self.retract_callback, callback_group=self.service_cb_group
        )

        # Define default values for our ROS parameters below #
        self.declare_parameter("DUMPER_MOTOR", 11)
        self.declare_parameter("DUMPER_POWER", 0.75)
        # Assign the ROS Parameters to member variables below #
        self.DUMPER_MOTOR = self.get_parameter("DUMPER_MOTOR").value
        self.DUMPER_POWER = self.get_parameter("DUMPER_POWER").value

        # Print the ROS Parameters to the terminal below #
        self.get_logger().info("DUMPER_MOTOR has been set to: " + str(self.DUMPER_MOTOR))

        # Current state of the dumper
        self.extended_state = False

        # Dumper Current Threshold
        self.current_threshold = 0.3
        self.dumper_current = 0.0

        self.dumper_current_sub = self.create_subscription(Float32, "Dumper_Current", self.dumper_current_callback, 10)

    # Define subsystem methods here
    def set_power(self, dumper_power: float) -> None:
        """This method sets power to the dumper."""
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.DUMPER_MOTOR, value=-1*dumper_power)
        )

    def stop(self) -> None:
        """This method stops the dumper."""
        self.cli_motor_set.call_async(MotorCommandSet.Request(type="duty_cycle", can_id=self.DUMPER_MOTOR, value=0.0))

    def toggle(self) -> None:
        """This method toggles the dumper."""
        if not self.extended_state:
            self.extend_dumper()
        else:
            self.retract_dumper()

    # Define service callback methods here
    def set_power_callback(self, request, response):
        """This service request sets power to the dumper."""
        self.set_power(request.power)
        response.success = True
        return response

    def stop_callback(self, request, response):
        """This service request stops the dumper."""
        if self.long_service_running:
            self.cancel_current_srv = True
        self.stop()
        response.success = True
        return response

    def toggle_callback(self, request, response):
        """This service request toggles the dumper."""
        self.toggle()
        response.success = True
        return response

    def extend_dumper(self) -> None:
        self.get_logger().info("Extending the dumper")
        self.extended_state = True
        self.long_service_running = True
        self.set_power(self.DUMPER_POWER)
        lastPowerTime = time.time()
        # Wait 0.5 seconds after the dumper current goes below the threshold before stopping the motor
        while time.time() - lastPowerTime < 0.5:
            if self.cancel_current_srv:
                self.cancel_current_srv = False
                break
            # If the dumper current is not below the threshold, update the last power time
            if not self.dumper_current < self.current_threshold:
                lastPowerTime = time.time()
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
            # self.get_logger().info("time.time() - lastPowerTime is currently: " + str(time.time() - lastPowerTime))
        self.stop()
        self.long_service_running = False
        self.get_logger().info("Done extending the dumper")

    def extend_callback(self, request, response):
        """This service request extends the dumper"""
        self.extend_dumper()
        response.success = True
        return response

    def retract_dumper(self) -> None:
        self.get_logger().info("Retracting the dumper")
        self.extended_state = False
        self.long_service_running = True
        self.set_power(-self.DUMPER_POWER)
        lastPowerTime = time.time()
        # Wait 0.5 seconds after the dumper current goes below the threshold before stopping the motor
        while time.time() - lastPowerTime < 0.5:
            if self.cancel_current_srv:
                self.cancel_current_srv = False
                break
            # If the dumper current is not below the threshold, update the last power time
            if not self.dumper_current < self.current_threshold:
                lastPowerTime = time.time()
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
            # self.get_logger().info("time.time() - lastPowerTime is currently: " + str(time.time() - lastPowerTime))
        self.stop()
        self.long_service_running = False
        self.get_logger().info("Done retracting the dumper")

    def retract_callback(self, request, response):
        """This service request retracts the dumper"""
        self.retract_dumper()
        response.success = True
        return response

    def dumper_current_callback(self, msg):
        self.dumper_current = msg.data
        # self.get_logger().info("Dumper current: " + str(self.dumper_current))


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = DumperNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    node.get_logger().info("Initializing the Dumper subsystem!")
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
