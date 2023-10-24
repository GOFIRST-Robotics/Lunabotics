# This ROS 2 node contains code for the swerve drivetrain subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: October 2023

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
    def __init__(self, drive_motor, turning_motor):
        self.drive_motor_can_id = drive_motor
        self.turning_motor_can_id = turning_motor
        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")

    def set_power(self, power: float) -> None:
        self.power.call_async(
            MotorCommandSet.Request(type="duty_cycle")
        )
        pass  # TODO: Implement this method by calling the MotorCommandSet service (type="duty cycle")

    def set_angle(self, angle: float) -> None:
        self.angle.call_async(
            MotorCommandSet.Request(type="position")
        )
        pass  # TODO: Implement this method by calling the MotorCommandSet service (type="position")

    def get_absolute_angle(self) -> float:
        pass  # TODO: Implement this method, save this for later

    def set_state(self, power: float, angle: float) -> None:
        self.setPower(power)
        self.setAngle(angle)


# This class represents the drivetrain as a whole (4 swerve modules)
class DrivetrainNode(Node):
    def __init__(self):
        """Initialize the ROS 2 drivetrain node."""
        super().__init__("drivetrain")

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

        self.back_left = SwerveModule(self.back_left_drive, self.back_left_turn)
        self.front_left = SwerveModule(self.front_left_drive, self.front_left_turn)
        self.back_right = SwerveModule(self.back_right_drive, self.back_right_turn)
        self.front_right = SwerveModule(self.front_right_drive, self.front_right_turn)

    # Define subsystem methods here
    def drive(self, forward_power: float, horizontal_power: float, turning_power: float) -> None:
        """This method drives the robot with the desired forward power and turning power."""

        # TODO: This drive() method will need to be completely rewritten for swerve
        # Look up swerve drive kinematics equations and write code here to implement them
        # Essentially, we need to take the forward_power, horizontal_power, and turning_power
        # and compute the what the angles and powers of all 4 swerve modules should be

        left_power = forward_power - turning_power
        right_power = forward_power + turning_power

        # Desaturate the wheel speeds if needed
        greater_input = max(abs(left_power), abs(right_power))
        if greater_input > 1.0:
            left_power /= greater_input
            right_power /= greater_input

        # Multiply power by -1 to invert motor direction
        # TODO: Instead of directly calling the motor services like below, utilize the setPower() and setAngle()
        # methods of the SwerveModule objects
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.FRONT_LEFT_DRIVE, value=left_power * -1)
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.BACK_LEFT_DRIVE, value=left_power * -1)
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.FRONT_RIGHT_DRIVE, value=right_power * -1)
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.BACK_RIGHT_DRIVE, value=right_power * -1)
        )

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
        self.drive(
            request.forward_power, request.horizontal_power, request.turning_power
        )  # TODO: You will need to modify the Drive srv (service) type to add a horizontal_power parameter DONE
        response.success = 0  # indicates success
        return response

    # Define subscriber callback methods here
    def cmd_vel_callback(self, msg: Twist) -> None:
        """This method is called whenever a message is received on the cmd_vel topic."""
        self.drive(msg.linear.x, msg.linear.y, msg.angular.z)  # TODO: Check if this is correct


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
