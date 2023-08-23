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


class DrivetrainNode(Node):
    def __init__(self):
        """Initialize the ROS 2 drivetrain node."""
        super().__init__("drivetrain")
        
        # Define publishers and subscribers here
        self.motor_command_pub = self.create_publisher(MotorCommand, "motor_cmd", 10)
        
        # Define motor CAN IDs here
        self.FRONT_LEFT_DRIVE = 1
        self.BACK_LEFT_DRIVE = 4
        self.FRONT_RIGHT_DRIVE = 3
        self.BACK_RIGHT_DRIVE = 2

    def drive(self, forward_power, turning_power):
        """This method drives the robot with the desired forward power and turning power."""
        linear_power = clamp(forward_power, -1.0, 1.0)  # Clamp the linear power between -1.0 and 1.0
        angular_power = clamp(turning_power, -1.0, 1.0)  # Clamp the angular power between -1.0 and 1.0

        left_power = linear_power - angular_power
        right_power = linear_power + angular_power

        # Desaturate the wheel speeds if needed
        greater_input = max(abs(left_power), abs(right_power))
        if greater_input > 1.0:
            left_power /= greater_input
            right_power /= greater_input

        self.motor_command_pub.publish(MotorCommand(self.FRONT_LEFT_DRIVE, left_power * -1))   # Multiply by -1 to invert motor direction
        self.motor_command_pub.publish(MotorCommand(self.BACK_LEFT_DRIVE, left_power * -1))    # Multiply by -1 to invert motor direction
        self.motor_command_pub.publish(MotorCommand(self.FRONT_RIGHT_DRIVE, right_power * -1)) # Multiply by -1 to invert motor direction
        self.motor_command_pub.publish(MotorCommand(self.BACK_RIGHT_DRIVE, right_power * -1))  # Multiply by -1 to invert motor direction

    def stop(self):
        """This method stops the drivetrain."""
        self.drive(0.0, 0.0)


def main(args=None):
    """The main function."""
    rclpy.init(args=args)
    print("Initializing the Drivetrain subsystem!")

    node = DrivetrainNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
