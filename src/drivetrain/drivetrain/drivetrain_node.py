# This ROS 2 node contains code for the drivetrain subsystem of the robot.
# Original Author: Akshat Arinav <arina004@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2024 by Charlie & Ashton


# Import the ROS 2 module
import rclpy
from rclpy.node import Node

# Import ROS 2 formatted message types
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float32

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import Drive, MotorCommandSet
from std_srvs.srv import Trigger


class DrivetrainNode(Node):
    def __init__(self):
        """Initialize the ROS 2 drivetrain node."""
        super().__init__("drivetrain")

        # Define default values for our ROS parameters below #
        self.declare_parameter("FRONT_LEFT_DRIVE", 10)
        self.declare_parameter("FRONT_RIGHT_DRIVE", 9)
        self.declare_parameter("BACK_LEFT_DRIVE", 7)
        self.declare_parameter("BACK_RIGHT_DRIVE", 8)
        self.declare_parameter("GAZEBO_SIMULATION", False)
        self.declare_parameter("MAX_DRIVETRAIN_RPM", 10000)
        self.declare_parameter("DRIVETRAIN_TYPE", "velocity")
        self.declare_parameter("DIGGER_SAFETY_ZONE", 120)  # Measured in potentiometer units (0 to 1023)

        # Assign the ROS Parameters to member variables below #
        self.FRONT_LEFT_DRIVE = self.get_parameter("FRONT_LEFT_DRIVE").value
        self.FRONT_RIGHT_DRIVE = self.get_parameter("FRONT_RIGHT_DRIVE").value
        self.BACK_LEFT_DRIVE = self.get_parameter("BACK_LEFT_DRIVE").value
        self.BACK_RIGHT_DRIVE = self.get_parameter("BACK_RIGHT_DRIVE").value
        self.GAZEBO_SIMULATION = self.get_parameter("GAZEBO_SIMULATION").value
        self.MAX_DRIVETRAIN_RPM = self.get_parameter("MAX_DRIVETRAIN_RPM").value
        self.DRIVETRAIN_TYPE = self.get_parameter("DRIVETRAIN_TYPE").value
        self.DIGGER_SAFETY_ZONE = self.get_parameter("DIGGER_SAFETY_ZONE").value

        # Define publishers and subscribers here
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.lift_pose_subscription = self.create_subscription(Float32, "lift_pose", self.lift_pose_callback, 10)

        if self.GAZEBO_SIMULATION:
            self.gazebo_front_left_wheel_pub = self.create_publisher(Float64, "front_left_wheel/cmd_vel", 10)
            self.gazebo_back_left_wheel_pub = self.create_publisher(Float64, "back_left_wheel/cmd_vel", 10)
            self.gazebo_front_right_wheel_pub = self.create_publisher(Float64, "front_right_wheel/cmd_vel", 10)
            self.gazebo_back_right_wheel_pub = self.create_publisher(Float64, "back_right_wheel/cmd_vel", 10)

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")

        # Define services (methods callable from the outside) here
        self.srv_stop = self.create_service(Trigger, "drivetrain/stop", self.stop_callback)
        self.srv_drive = self.create_service(Drive, "drivetrain/drive", self.drive_callback)

        # Print the ROS Parameters to the terminal below #
        self.get_logger().info("FRONT_LEFT_DRIVE has been set to: " + str(self.FRONT_LEFT_DRIVE))
        self.get_logger().info("FRONT_RIGHT_DRIVE has been set to: " + str(self.FRONT_RIGHT_DRIVE))
        self.get_logger().info("BACK_LEFT_DRIVE has been set to: " + str(self.BACK_LEFT_DRIVE))
        self.get_logger().info("BACK_RIGHT_DRIVE has been set to: " + str(self.BACK_RIGHT_DRIVE))
        self.get_logger().info("GAZEBO_SIMULATION has been set to: " + str(self.GAZEBO_SIMULATION))
        self.get_logger().info("MAX_DRIVETRAIN_RPM has been set to: " + str(self.MAX_DRIVETRAIN_RPM))
        self.get_logger().info("DIGGER_SAFETY_ZONE has been set to: " + str(self.DIGGER_SAFETY_ZONE))

        # Current position of the lift motor in potentiometer units (0 to 1023)
        self.current_lift_position = None  # We don't know the current position yet

    # Define subsystem methods here
    def drive(self, forward_power: float, turning_power: float) -> None:
        """This method drives the robot with the desired forward and turning power."""

        if (
            self.current_lift_position is None or self.current_lift_position > self.DIGGER_SAFETY_ZONE
        ) and not self.GAZEBO_SIMULATION:
            self.get_logger().warn("Digger outside the safety zone, cannot drive!")
            self.stop()
            return

        # Clamp the values between -1 and 1
        forward_power = max(-1.0, min(forward_power, 1.0))
        turning_power = max(-1.0, min(turning_power, 1.0))

        # Calculate the wheel speeds for each side of the drivetrain
        leftPower = forward_power - turning_power
        rightPower = forward_power + turning_power

        # Desaturate the wheel speeds if needed
        if abs(leftPower) > 1.0 or abs(rightPower) > 1.0:
            scale_factor = 1.0 / max(abs(leftPower), abs(rightPower))
            leftPower *= scale_factor
            rightPower *= scale_factor

        if self.DRIVETRAIN_TYPE == "velocity":
            leftPower *= self.MAX_DRIVETRAIN_RPM
            rightPower *= self.MAX_DRIVETRAIN_RPM

        # Send velocity (not duty cycle) motor commands to the motor_control_node
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(can_id=self.FRONT_LEFT_DRIVE, type=self.DRIVETRAIN_TYPE, value=leftPower)
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(can_id=self.BACK_LEFT_DRIVE, type=self.DRIVETRAIN_TYPE, value=leftPower)
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(can_id=self.FRONT_RIGHT_DRIVE, type=self.DRIVETRAIN_TYPE, value=rightPower)
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(can_id=self.BACK_RIGHT_DRIVE, type=self.DRIVETRAIN_TYPE, value=rightPower)
        )

        # Publish the wheel speeds to the gazebo simulation
        if self.GAZEBO_SIMULATION:
            if self.DRIVETRAIN_TYPE == "velocity":
                self.gazebo_front_left_wheel_pub.publish(Float64(data=leftPower / self.MAX_DRIVETRAIN_RPM))
                self.gazebo_back_left_wheel_pub.publish(Float64(data=leftPower / self.MAX_DRIVETRAIN_RPM))
                self.gazebo_front_right_wheel_pub.publish(Float64(data=rightPower / self.MAX_DRIVETRAIN_RPM))
                self.gazebo_back_right_wheel_pub.publish(Float64(data=rightPower / self.MAX_DRIVETRAIN_RPM))
            else:
                self.gazebo_front_left_wheel_pub.publish(Float64(data=leftPower))
                self.gazebo_back_left_wheel_pub.publish(Float64(data=leftPower))
                self.gazebo_front_right_wheel_pub.publish(Float64(data=rightPower))
                self.gazebo_back_right_wheel_pub.publish(Float64(data=rightPower))
        return True

    def stop(self) -> None:
        """This method stops the drivetrain."""
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(can_id=self.FRONT_LEFT_DRIVE, type="duty_cycle", value=0.0)
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(can_id=self.BACK_LEFT_DRIVE, type="duty_cycle", value=0.0)
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(can_id=self.FRONT_RIGHT_DRIVE, type="duty_cycle", value=0.0)
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(can_id=self.BACK_RIGHT_DRIVE, type="duty_cycle", value=0.0)
        )

    # Define service callback methods here

    def stop_callback(self, request, response):
        """This service request stops the drivetrain."""
        self.stop()
        response.success = True
        return response

    def drive_callback(self, request, response):
        """This service request drives the robot with the specified speeds."""
        response.success = self.drive(request.forward_power, request.turning_power)
        return response

    # Define subscriber callback methods here

    def cmd_vel_callback(self, msg: Twist) -> None:
        """This method is called whenever a message is received on the cmd_vel topic."""
        self.drive(msg.linear.x, msg.angular.z)

    # Define the subscriber callback for the lift pose topic
    def lift_pose_callback(self, msg: Float32):
        # Average the two potentiometer values
        self.current_lift_position = msg.data


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
