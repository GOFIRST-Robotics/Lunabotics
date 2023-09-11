# This ROS 2 node contains code for the digger subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2023

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node

# Import Python Modules
import serial  # Serial communication with the Arduino. Install with: <sudo pip3 install pyserial>

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet
from rovr_interfaces.srv import SetPower, Stop, LinearActuator


class DiggerNode(Node):
    def __init__(self):
        """Initialize the ROS 2 digger node."""
        super().__init__("digger")

        # Try connecting to the Arduino over Serial
        try:
            # Set this as a static Serial port!
            self.arduino = serial.Serial("/dev/Arduino_Uno", 9600)  # 9600 is the baud rate
        except Exception as e:
            self.get_logger().error(str(e))  # If an exception is raised, log it, and then move on

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")

        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(SetPower, "digger/toggle", self.toggle_callback)
        self.srv_stop = self.create_service(Stop, "digger/stop", self.stop_callback)
        self.srv_setPower = self.create_service(SetPower, "digger/setPower", self.set_power_callback)
        self.srv_stop_linear_actuator = self.create_service(
            Stop, "digger/stop_linear_actuator", self.stop_linear_actuator_callback
        )
        self.srv_extend = self.create_service(LinearActuator, "digger/extend", self.extend_callback)
        self.srv_retract = self.create_service(LinearActuator, "digger/retract", self.retract_callback)
        self.srv_read_all = self.create_service(Stop, "digger/read_all", self.read_all_callback)

        # Define motor CAN IDs here
        self.DIGGER = 8

        # Current subsystem state
        self.running = False

    # Define digging drum methods here
    def set_power(self, power: float) -> None:
        """This method sets power to the digging drum."""
        self.running = True
        self.cli_motor_set.call_async(MotorCommandSet.Request(type="duty_cycle", can_id=self.DIGGER, value=power))

    def stop(self) -> None:
        """This method stops the digging drum."""
        self.running = False
        self.cli_motor_set.call_async(MotorCommandSet.Request(type="duty_cycle", can_id=self.DIGGER, value=0.0))

    def toggle(self, power: float) -> None:
        """This method toggles the digging drum."""
        if self.running:
            self.stop()
        else:
            self.set_power(power)

    # Define linear actuator methods here
    def extend(self, power: int, wait: bool = False):
        """This method extends the linear actuator."""
        self.arduino.write(f"e{chr(power)}".encode("ascii"))
        # TODO: This wait until done case doesn't seem to be working
        if wait:  # Wait for a confirmation message from the Arduino (if we want to)
            reading = self.arduino.read()
            while reading != b"f":  # this is just the character we arbitrarily chose in the Arduino code:
                reading = self.arduino.read()

    def retract(self, power: int, wait: bool = False):
        """This method retracts the linear actuator."""
        self.arduino.write(f"r{chr(power)}".encode("ascii"))
        # TODO: This wait until done case doesn't seem to be working
        if wait:  # Wait for a confirmation message from the Arduino (if we want to)
            reading = self.arduino.read()
            while reading != b"s":  # this is just the character we arbitrarily chose in the Arduino code
                reading = self.arduino.read()

    def stop_linear_actuator(self):
        """This method stops the linear actuator."""
        self.arduino.write(f"e{chr(0)}".encode("ascii"))

    def read_all(self):
        """This method reads all messages from the serial buffer to clear them out."""
        self.arduino.read_all()

    # Define digging drum service callback methods here
    def set_power_callback(self, request, response) -> None:
        """This service request sets power to the digging drum."""
        self.set_power(request.power)
        response.success = 0  # indicates success
        return response

    def stop_callback(self, request, response) -> None:
        """This service request stops the digging drum."""
        self.stop()
        response.success = 0  # indicates success
        return response

    def toggle_callback(self, request, response) -> None:
        """This service request toggles the digging drum."""
        self.toggle(request.power)
        response.success = 0  # indicates success
        return response

    # Define linear actuator service callback methods here
    def stop_linear_actuator_callback(self, request, response) -> None:
        """This service request stops the digging drum."""
        self.stop_linear_actuator()
        response.success = 0  # indicates success
        return response

    def extend_callback(self, request, response) -> None:
        """This service request extends the linear actuator."""
        self.extend(request.power, request.wait)
        response.success = 0  # indicates success
        return response

    def retract_callback(self, request, response) -> None:
        """This service request retracts the linear actuator."""
        self.retract(request.power, request.wait)
        response.success = 0  # indicates success
        return response

    def read_all_callback(self, request, response) -> None:
        """This service request reads all messages from the serial buffer."""
        self.read_all()
        response.success = 0  # indicates success
        return response


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = DiggerNode()
    node.get_logger().info("Initializing the Digger subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
