# This ROS 2 node contains code for the dumper subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2024
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2024

# Import the ROS 2 Python module
import time
import rclpy
from rclpy.node import Node

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet, MotorCommandGet, SetPower
from rovr_interfaces.msg import LimitSwitches
from std_srvs.srv import Trigger


class DumperNode(Node):
    def __init__(self):
        """Initialize the ROS 2 dumper node."""
        super().__init__("dumper")

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(SetPower, "dumper/toggle", self.toggle_callback)
        self.srv_stop = self.create_service(Trigger, "dumper/stop", self.stop_callback)
        self.srv_setPower = self.create_service(SetPower, "dumper/setPower", self.set_power_callback)

        self.srv_extendDumper = self.create_service(Trigger, "dumper/extendDumper", self.extend_callback)
        self.srv_retractDumper = self.create_service(Trigger, "dumper/retractDumper", self.retract_callback)
        self.srv_dump = self.create_service(Trigger, "dumper/dump", self.dump_callback)

        # Define default values for our ROS parameters below #
        self.declare_parameter("DUMPER_MOTOR", 11)
        self.declare_parameter("DUMPER_POWER", 0.5)
        # Assign the ROS Parameters to member variables below #
        self.DUMPER_MOTOR = self.get_parameter("DUMPER_MOTOR").value
        self.DUMPER_POWER = self.get_parameter("DUMPER_POWER").value

        # Print the ROS Parameters to the terminal below #
        self.get_logger().info("DUMPER_MOTOR has been set to: " + str(self.DUMPER_MOTOR))

        # Current state of the dumper
        self.running = False

        self.limit_switch_sub = self.create_subscription(LimitSwitches, "limitSwitches", self.limit_switch_callback, 10)

    # Define subsystem methods here
    def set_power(self, dumper_power: float) -> None:
        """This method sets power to the dumper."""
        self.running = True
        if dumper_power > 0 and self.top_limit_pressed:
            self.get_logger().warn("WARNING: Top limit switch pressed!")
            self.stop()  # Stop the dumper
            return
        elif dumper_power < 0 and self.bottom_limit_pressed:
            self.get_logger().warn("WARNING: Top limit switch pressedF!")
            self.stop()  # stop the dumper
            return
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.DUMPER_MOTOR, value=dumper_power)
        )

    def stop(self) -> None:
        """This method stops the dumper."""
        self.running = False
        self.cli_motor_set.call_async(MotorCommandSet.Request(type="duty_cycle", can_id=self.DUMPER_MOTOR, value=0.0))

    def toggle(self, dumper_power: float) -> None:
        """This method toggles the dumper."""
        if self.running:
            self.stop()
        else:
            self.set_power(dumper_power)

    # Define service callback methods here
    def set_power_callback(self, request, response):
        """This service request sets power to the dumper."""
        self.set_power(request.power)
        response.success = True
        return response

    def stop_callback(self, request, response):
        """This service request stops the dumper."""
        self.stop()
        response.success = True
        return response

    def toggle_callback(self, request, response):
        """This service request toggles the dumper."""
        self.toggle(request.power)
        response.success = True
        return response

    def extend_dumper(self) -> None:
        while not self.top_limit_pressed:
            self.set_power(self.DUMPER_POWER)
        self.stop()

    def extend_callback(self, request, response):
        """This service request extends the dumper"""
        self.extend_dumper()
        response.success = True
        return response

    def retract_dumper(self) -> None:
        while not self.bottom_limit_pressed:
            self.set_power(-self.DUMPER_POWER)
        self.stop()

    def retract_callback(self, request, response):
        """This service request retracts the dumper"""
        self.retract_dumper()
        response.success = True
        return response

    def dump(self) -> None:
        # extend the dumper
        self.extend_dumper()
        # wait for 5 seconds before retracting the dumper
        time.sleep(5)
        # retract the dumper
        self.retract_dumper()

    def dump_callback(self, request, response):
        self.dump()
        response.success = True
        return response

    def limit_switch_callback(self, msg: LimitSwitches):
        """This subscriber callback method is called whenever a message is received on the limitSwitches topic."""
        if not self.top_limit_pressed and msg.top_limit_switch:
            self.stop()  # Stop the lift system
        if not self.bottom_limit_pressed and msg.bottom_limit_switch:
            self.stop()  # Stop the lift system
        self.top_limit_pressed = msg.top_limit_switch
        self.bottom_limit_pressed = msg.bottom_limit_switch


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = DumperNode()
    node.get_logger().info("Initializing the Dumper subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
