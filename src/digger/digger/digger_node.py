# This ROS 2 node contains code for the digger subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2023

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node

# Import custom ROS 2 interfaces
from rovr_interfaces.msg import MotorCommand
from rovr_interfaces.srv import DiggerToggle
from rovr_interfaces.srv import DiggerStop
from rovr_interfaces.srv import DiggerSetPower

# Define helper functions here
def clamp(number: float, minimum: float, maximum: float) -> float:
    """Clamps a number between the specified minimum and maximum."""
    return max(minimum, min(number, maximum))


class DiggerNode(Node):
    def __init__(self):
        """Initialize the ROS 2 digger node."""
        super().__init__("digger")
        
        # Define publishers and subscribers here
        self.motor_command_pub = self.create_publisher(MotorCommand, "motor_cmd", 10)
        
        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(DiggerToggle, 'digger/toggle', self.toggle_callback)
        self.srv_stop = self.create_service(DiggerStop, 'digger/stop', self.stop_callback)
        self.srv_setPower = self.create_service(DiggerSetPower, 'digger/setPower', self.set_power_callback)
        
        # Define motor CAN IDs here
        self.DIGGER = 8
        
        # Current subsystem state
        self.running = False

    # Define subsystem methods here
    def set_power(self, power: float) -> None:
        """This method sets power to the digging drum."""
        self.running = True
        p = clamp(power, -1.0, 1.0)  # Clamp the power between -1.0 and 1.0
        self.motor_command_pub.publish(MotorCommand(can_id=self.DIGGER, type="duty_cycle", value=p))

    def stop(self) -> None:
        """This method stops the digging drum."""
        self.running = False
        self.motor_command_pub.publish(MotorCommand(can_id=self.DIGGER, type="duty_cycle", value=0.0))

    def toggle(self, power: float) -> None:
        """This method toggles the digging drum."""
        if self.running:
            self.stop()
        else:
            self.set_power(power)
            
    # Define service callback methods here
    def set_power_callback(self, request, response) -> None:
        """This service request sets power to the digging drum."""
        self.set_power(request.power)
        response.success = 1 # indicate success
        return response

    def stop_callback(self, request, response) -> None:
        """This service request stops the digging drum."""
        self.stop()
        response.success = 1 # indicate success
        return response

    def toggle_callback(self, request, response) -> None:
        """This service request toggles the digging drum."""
        self.toggle(request.power)
        response.success = 1 # indicate success
        return response
    

def main(args=None):
    """The main function."""
    rclpy.init(args=args)
    print("Initializing the Digger subsystem!")

    node = DiggerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
