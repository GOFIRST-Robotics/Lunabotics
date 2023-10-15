# This ROS 2 node contains code for the conveyor subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2023

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node
import serial  # Allows for serial communication with an Arduino. Install with "sudo pip3 install pyserial".

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet
from rovr_interfaces.srv import SetPower, Stop


class ConveyorNode(Node):
    def __init__(self):
        """Initialize the ROS 2 conveyor node."""
        super().__init__("conveyor")

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")

        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(SetPower, "conveyor/toggle", self.toggle_callback)
        self.srv_stop = self.create_service(Stop, "conveyor/stop", self.stop_callback)
        self.srv_setPower = self.create_service(SetPower, "conveyor/setPower", self.set_power_callback)

        # Define motor CAN IDs here
        self.CONVEYOR_BELT_MOTOR = 6

        # Current subsystem state
        self.running = False

        # For now, we will assume that we are using an Arduino to send PWM commands to the linear actuator(s)
        try:  # Try connecting to the Arduino over Serial
            # Set "/dev/Arduino_Uno" as a static Serial port!
            self.arduino = serial.Serial("/dev/Arduino_Uno", 9600, timeout=0.01)
        except Exception as e:
            print(e)  # If an exception is raised, print it, and then move on

    # Define Arduino helper methods here
    def clear_serial_buffer(self) -> None:
        self.arduino.read_all()
        
    def read_serial(self) -> str:
        return self.arduino.read().decode("ascii")
    
    def write_serial(self, msg: str) -> None:
        self.arduino.write(msg.encode("ascii"))

    # Define subsystem methods here
    def set_power(self, conveyor_power: float) -> None:
        """This method sets power to the conveyor belts."""
        self.running = True

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.CONVEYOR_BELT_MOTOR, value=conveyor_power)
        )

    def stop(self) -> None:
        """This method stops both conveyor belts."""
        self.running = False

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.CONVEYOR_BELT_MOTOR, value=0.0)
        )

    def toggle(self, conveyor_belt_power: float) -> None:
        """This method toggles the conveyor belts."""
        if self.running:
            self.stop()
        else:
            self.set_power(conveyor_belt_power)

    def set_incline_angle(self, angle: float) -> None:
        pass  # TODO: Implement this method by sending a command to the Arduino and then waiting for a response

    def set_height(self, height: float) -> None:
        pass  # TODO: Implement this method by sending a command to the Arduino and then waiting for a response

    # Define service callback methods here
    def set_power_callback(self, request, response):
        """This service request sets power to the conveyor belts."""
        self.set_power(request.conveyor_belt_power)
        response.success = 0  # indicates success
        return response

    def stop_callback(self, request, response):
        """This service request stops the conveyor belts."""
        self.stop()
        response.success = 0  # indicates success
        return response

    def toggle_callback(self, request, response):
        """This service request toggles the conveyor belts."""
        self.toggle(request.conveyor_belt_power)
        response.success = 0  # indicates success
        return response

    def set_incline_callback(self, request, response):
        # TODO: Implement this callback wrapper
        response.success = 0  # indicates success
        return response

    def set_height_callback(self, request, response):
        # TODO: Implement this callback wrapper
        response.success = 0  # indicates success
        return response


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = ConveyorNode()
    node.get_logger().info("Initializing the Conveyor subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
