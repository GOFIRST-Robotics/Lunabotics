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
        
        # Current state
        self.running = False

    def set_power(self, drum_belt_power: float, conveyor_belt_power: float) -> None:
        """This method sets power to the conveyor belts."""
        self.running = True
        drum_power = clamp(drum_belt_power, -1.0, 1.0)  # Clamp the power between -1.0 and 1.0
        conveyor_power = clamp(conveyor_belt_power, -1.0, 1.0)  # Clamp the power between -1.0 and 1.0
        self.motor_command_pub.publish(MotorCommand(self.DRUM_BELT_MOTOR, drum_power))
        self.motor_command_pub.publish(MotorCommand(self.CONVEYOR_BELT_MOTOR, conveyor_power))

    def stop(self) -> None:
        """This method stops both conveyor belts."""
        self.running = False
        self.motor_command_pub.publish(MotorCommand(self.DRUM_BELT_MOTOR, 0.0))
        self.motor_command_pub.publish(MotorCommand(self.CONVEYOR_BELT_MOTOR, 0.0))
        
    def toggle(self, drum_belt_power: float, conveyor_belt_power: float) -> None:
        """This method toggles the conveyor belts."""
        if self.running:
            self.stop()
        else:
            self.set_power(drum_belt_power, conveyor_belt_power)


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