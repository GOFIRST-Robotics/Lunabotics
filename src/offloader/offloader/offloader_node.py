# This ROS 2 node contains code for the offloader subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2023

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node

# Import custom ROS 2 interfaces
from rovr_interfaces.msg import MotorCommand
from rovr_interfaces.srv import OffloaderToggle, OffloaderSetPower, Stop

# Define helper functions here
def clamp(number: float, minimum: float, maximum: float) -> float:
    """Clamps a number between the specified minimum and maximum."""
    return max(minimum, min(number, maximum))


class OffloaderNode(Node):
    def __init__(self):
        """Initialize the ROS 2 offloader node."""
        super().__init__("offloader")
        
        # Define publishers and subscribers here
        self.motor_command_pub = self.create_publisher(MotorCommand, "motor_cmd", 10)
        
        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(OffloaderToggle, 'offloader/toggle', self.toggle_callback)
        self.srv_stop = self.create_service(Stop, 'offloader/stop', self.stop_callback)
        self.srv_setPower = self.create_service(OffloaderSetPower, 'offloader/setPower', self.set_power_callback)
        
        # Define motor CAN IDs here
        self.OFFLOADER = 5
        
        # Current subsystem state
        self.running = False

    # Define subsystem methods here
    def set_power(self, power: float) -> None:
        """This method sets power to the offloading belt."""
        self.running = True
        p = clamp(power, -1.0, 1.0)  # Clamp the power between -1.0 and 1.0
        self.motor_command_pub.publish(MotorCommand(can_id=self.OFFLOADER, type="duty_cycle", value=p))

    def stop(self) -> None:
        """This method stops the offloading belt."""
        self.running = False
        self.motor_command_pub.publish(MotorCommand(can_id=self.OFFLOADER, type="duty_cycle", value=0.0))

    def toggle(self, power: float) -> None:
        """This method toggles the offloading belt."""
        if self.running:
            self.stop()
        else:
            self.set_power(power)
            
    # Define service callback methods here
    def set_power_callback(self, request, response) -> None:
        """This service request sets power to the offloading belt."""
        self.set_power(request.power)
        response.success = 1 # indicates success
        return response

    def stop_callback(self, request, response) -> None:
        """This service request stops the offloading belt."""
        self.stop()
        response.success = 1 # indicates success
        return response

    def toggle_callback(self, request, response) -> None:
        """This service request toggles the offloading belt."""
        self.toggle(request.power)
        response.success = 1 # indicates success
        return response
    

def main(args=None):
    """The main function."""
    rclpy.init(args=args)
    print("Initializing the Offloader subsystem!")

    node = OffloaderNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
