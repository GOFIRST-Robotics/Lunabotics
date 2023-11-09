# This ROS 2 node contains code for the swerve drivetrain subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: October 2023

import math 

# Import the ROS 2 module
import rclpy
from rclpy.node import Node

# Import ROS 2 formatted message types
from geometry_msgs.msg import Twist

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet
from rovr_interfaces.srv import Stop, Drive


# This class represents an individual swerve module
class SwerveModule:
    def __init__(self, drive_motor, turning_motor, motor_set):
        self.drive_motor_can_id = drive_motor
        self.turning_motor_can_id = turning_motor
        self.cli_motor_set = motor_set


    def set_power(self, power: float) -> None:
        self.cli_motor_set.call_async(MotorCommandSet.Request(type="duty_cycle", value=power))

    def set_angle(self, angle: float) -> None:
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="position",
                value=angle,
            )
        )

    def get_absolute_angle(self) -> float:
        pass  # TODO: Implement this method for reading the absolute encoder (save this for last)

    def set_state(self, power: float, angle: float) -> None:
        self.setPower(power)
        self.setAngle(angle)


# This class represents the drivetrain as a whole (4 swerve modules)
class DrivetrainNode(Node):
    def __init__(self):
        """Initialize the ROS 2 drivetrain node."""
        super().__init__("drivetrain")
        #Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        
        # Define publishers and subscribers here
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # Define services (methods callable from the outside) here
        self.srv_stop = self.create_service(Stop, "drivetrain/stop", self.stop_callback)
        self.srv_drive = self.create_service(Drive, "drivetrain/drive", self.drive_callback)

        # Assigning can_id motor to each swerve module
        self.back_left_drive = 1
        self.back_left_turn = 2
        self.front_left_drive = 3
        self.front_left_turn = 4
        self.back_right_drive = 5
        self.back_right_turn = 6
        self.front_right_drive = 7
        self.front_right_turn = 8

        # Create each swerve module using
        self.back_left = SwerveModule(self.back_left_drive, self.back_left_turn, self.cli_motor_set)
        self.front_left = SwerveModule(self.front_left_drive, self.front_left_turn, self.cli_motor_set)
        self.back_right = SwerveModule(self.back_right_drive, self.back_right_turn, self.cli_motor_set)
        self.front_right = SwerveModule(self.front_right_drive, self.front_right_turn, self.cli_motor_set)

    # Define subsystem methods here
    def drive(self, forward_power: float, horizontal_power: float, turning_power: float) -> None:
        """This method drives the robot with the desired forward, horizontal and turning power."""

        # Essentially, we need to take the forward_power (y), horizontal_power(x), and turning_power (z)
        # and compute the what the angles and powers of all 4 swerve modules should be.

        # Vector layouts = [Drive Power, Drive Direction(Degrees from forwards going counterclockwise)] 

        #TODO: Get specifc measurements for wheel_base and track_width
        wheel_base = 1     #half of the wheelbase length, change value later
        track_width = 1  #half of the trackwidth length, change value later

        #Writing equations to simplify expressions
        A = horizontal_power - turning_power*wheel_base 
        B = horizontal_power + turning_power*wheel_base
        C = forward_power - turning_power*track_width
        D = forward_power + turning_power*track_width

        #Gives speed and angle for each module
        back_left_vector = [math.sqrt(A**2 + D**2), math.atan2(A,D)*180/math.pi]
        front_left_vector = [math.sqrt(B**2 + D**2), math.atan2(B,D)*180/math.pi]
        back_right_vector = [math.sqrt(A**2 + C**2), math.atan2(A,C)*180/math.pi]
        front_right_vector = [math.sqrt(B**2 + C**2), math.atan2(B,C)*180/math.pi]

        #Normalizing speeds
        largest_power = max([back_left_vector[0], front_left_vector[0], back_right_vector[0], front_right_vector[0]])
        if largest_power > 1.0:
            back_left_vector[0] = back_left_vector[0]/largest_power
            front_left_vector[0] = front_left_vector[0]/largest_power
            back_right_vector[0] = back_right_vector[0]/largest_power
            front_right_vector[0] = front_right_vector[0]/largest_power 

        # TODO: optimize turning
        self.back_left.set_power(back_left_vector[0])
        self.front_left.set_power(front_left_vector[0])
        self.back_right.set_power(back_right_vector[0])
        self.front_right.set_power(front_right_vector[0])
        
        self.back_left.set_angle(back_left_vector[1])
        self.front_left.set_angle(front_left_vector[1])
        self.back_right.set_angle(back_right_vector[1])
        self.front_right.set_angle(front_right_vector[1])

        
    def stop(self) -> None:
        """This method stops the drivetrain."""
        self.drive(0.0, 0.0, 0.0)

    # Define service callback methods here
    def stop_callback(self, request, response):
        """This service request stops the drivetrain."""
        self.stop()
        response.success = 0  # indicates success
        return response

    def drive_callback(self, request, response):
        """This service request drives the robot with the specified speeds."""
        self.drive(request.forward_power, request.horizontal_power, request.turning_power)
        response.success = 0  # indicates success
        return response

    # Define subscriber callback methods here
    def cmd_vel_callback(self, msg: Twist) -> None:
        """This method is called whenever a message is received on the cmd_vel topic."""
        self.drive(msg.linear.y, msg.linear.x, msg.angular.z)  


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = DrivetrainNode()
    node.get_logger().info("Initializing the Drivetrain subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
