# This ROS 2 node contains code for the dumper subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2024
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2024

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet, MotorCommandGet
from rovr_interfaces.srv import SetPower, Stop


class DumperNode(Node):
    def __init__(self):
        """Initialize the ROS 2 dumper node."""
        super().__init__("dumper")

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(SetPower, "dumper/toggle", self.toggle_callback)
        self.srv_stop = self.create_service(Stop, "dumper/stop", self.stop_callback)
        self.srv_setPower = self.create_service(SetPower, "dumper/setPower", self.set_power_callback)

        # Define default values for our ROS parameters below #
        self.declare_parameter("DUMPER_MOTOR", 11)

        # Assign the ROS Parameters to member variables below #
        self.DUMPER_MOTOR = self.get_parameter("DUMPER_MOTOR").value

        # Print the ROS Parameters to the terminal below #
        self.get_logger().info("DUMPER_MOTOR has been set to: " + str(self.DUMPER_MOTOR))

        # Current state of the dumper
        self.running = False

    # Define subsystem methods here
    def set_power(self, dumper_power: float) -> None:
        """This method sets power to the dumper."""
        self.running = True
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
        response.success = 0  # indicates success
        return response

    def stop_callback(self, request, response):
        """This service request stops the dumper."""
        self.stop()
        response.success = 0  # indicates success
        return response

    def toggle_callback(self, request, response):
        """This service request toggles the dumper."""
        self.toggle(request.power)
        response.success = 0  # indicates success
        return response


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