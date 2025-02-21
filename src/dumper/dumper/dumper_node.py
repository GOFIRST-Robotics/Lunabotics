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
from rovr_interfaces.msg import LimitSwitches
from std_srvs.srv import Trigger


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
        self.declare_parameter("DUMPER_POWER", 0.5)
        # Assign the ROS Parameters to member variables below #
        self.DUMPER_MOTOR = self.get_parameter("DUMPER_MOTOR").value
        self.DUMPER_POWER = self.get_parameter("DUMPER_POWER").value

        # Print the ROS Parameters to the terminal below #
        self.get_logger().info("DUMPER_MOTOR has been set to: " + str(self.DUMPER_MOTOR))

        # Current state of the dumper
        self.extended_state = False

        self.dumper_duty_cycle = 0.0

        self.dumper_duty_cycle_sub = self.create_subscription(float, "dutyCycles", self.dumper_duty_cycle_callback, 10)

    # Define subsystem methods here
    def set_power(self, dumper_power: float) -> None:
        """This method sets power to the dumper."""
        if dumper_power > 0 and self.dumper_duty_cycle == 0.0:
            self.get_logger().warn("WARNING: Duty cycle is 0.0!")
            self.stop()  # Stop the dumper
        elif dumper_power < 0 and self.dumper_duty_cycle == 0.0:
            self.get_logger().warn("WARNING: Duty cycle is 0.0!")
            self.stop()  # stop the dumper
        else:
            self.cli_motor_set.call_async(
                MotorCommandSet.Request(type="duty_cycle", can_id=self.DUMPER_MOTOR, value=dumper_power)
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
        while not self.dumper_duty_cycle == 0.0:
            if self.cancel_current_srv:
                self.cancel_current_srv = False
                break
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
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
        while not self.dumper_duty_cycle == 0.0:
            if self.cancel_current_srv:
                self.cancel_current_srv = False
                break
            time.sleep(0.1)  # We don't want to spam loop iterations too fast
        self.stop()
        self.long_service_running = False
        self.get_logger().info("Done retracting the dumper")

    def retract_callback(self, request, response):
        """This service request retracts the dumper"""
        self.retract_dumper()
        response.success = True
        return response

    def dumper_duty_cycle_callback(self, duty_cycle_msg):
        if not self.dumper_duty_cycle == 0.0 and duty_cycle_msg.data == 0.0:
            self.stop()
        self.dumper_duty_cycle = duty_cycle_msg.data


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
