# This node contains code for the drivetrain subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: August 2023

# Import the ROS 2 module
import rclpy
from rclpy.node import Node

# Import custom ROS 2 message type
from rovr_interfaces.msg import MotorCommand


def clamp(number, minimum, maximum):
    """Clamps a number between the specified minimum and maximum."""
    return max(minimum, min(number, maximum))


class DiggerNode(Node):
    def __init__(self):
        """Initialize the ROS 2 digger node."""
        super().__init__("digger")
        
        # Define publishers and subscribers here
        self.motor_command_pub = self.create_publisher(MotorCommand, "motor_cmd", 10)
        
        # Define motor CAN IDs here
        self.DIGGER = 8
        
        # Current state
        self.running = False

    def set_power(self, power: float) -> None:
        """This method sets power to the digging drum."""
        self.running = True
        p = clamp(power, -1.0, 1.0)  # Clamp the power between -1.0 and 1.0
        self.motor_command_pub.publish(MotorCommand(can_id=self.DIGGER, type="duty_cycle", value=p))

    def stop(self) -> None:
        """This method stops the digging drum."""
        self.running = False
        self.motor_command_pub.publish(MotorCommand(can_id=self.DIGGER, type="duty_cycle", value=0.0))
        
    def toggle(self, drum_power: float) -> None:
        """This method toggles the digging drum."""
        if self.running:
            self.stop()
        else:
            self.set_power(drum_power)


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
