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


class ConveyorNode(Node):
    def __init__(self):
        """Initialize the ROS 2 conveyor node."""
        super().__init__("conveyor")
        
        # Define publishers and subscribers here
        self.motor_command_pub = self.create_publisher(MotorCommand, "motor_cmd", 10)
        
        # Define motor CAN IDs here
        self.DRUM_BELT_MOTOR = 7
        self.CONVEYOR_BELT_MOTOR = 6

    def set_power(self, power):
        """This method drives the robot with the desired forward power and turning power."""
        p = clamp(power, -1.0, 1.0)  # Clamp the power between -1.0 and 1.0
        self.motor_command_pub.publish(MotorCommand(self.DRUM_BELT_MOTOR, p))
        self.motor_command_pub.publish(MotorCommand(self.CONVEYOR_BELT_MOTOR, p))

    def stop(self):
        """This method stops the conveyor."""
        self.set_power(0.0)


def main(args=None):
    """The main function."""
    rclpy.init(args=args)
    print("Initializing the Conveyor subsystem!")

    node = ConveyorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
