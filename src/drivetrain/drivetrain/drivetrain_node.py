# This ROS 2 node contains code for the drivetrain subsystem of the robot.
# Original Author: Akshat Arinav <arina004@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2024 by Charlie & Ashton


# Import the ROS 2 module
import rclpy
from rclpy.node import Node

# Import ROS 2 formatted message types
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import Drive, MotorCommandSet
from std_srvs.srv import Trigger
from rovr_interfaces.msg import LimitSwitches


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

        # Assign the ROS Parameters to member variables below #
        self.FRONT_LEFT_DRIVE = self.get_parameter("FRONT_LEFT_DRIVE").value
        self.FRONT_RIGHT_DRIVE = self.get_parameter("FRONT_RIGHT_DRIVE").value
        self.BACK_LEFT_DRIVE = self.get_parameter("BACK_LEFT_DRIVE").value
        self.BACK_RIGHT_DRIVE = self.get_parameter("BACK_RIGHT_DRIVE").value
        self.GAZEBO_SIMULATION = self.get_parameter("GAZEBO_SIMULATION").value
        self.MAX_DRIVETRAIN_RPM = self.get_parameter("MAX_DRIVETRAIN_RPM").value

        # State variable for the drivetrain based on the digger's position
        self.ready_to_drive = True

        # Define publishers and subscribers here
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.limit_switch_reader = self.create_subscription(
            LimitSwitches, "limitSwitches", self.limit_switch_callback, 10
        )

        if self.GAZEBO_SIMULATION:
            self.gazebo_wheelFrontLeft_pub = self.create_publisher(Float64, "wheelFrontLeft/cmd_vel", 10)
            self.gazebo_wheelFrontRight_pub = self.create_publisher(Float64, "wheelFrontRight/cmd_vel", 10)
            self.gazebo_wheelBackLeft_pub = self.create_publisher(Float64, "wheelBackLeft/cmd_vel", 10)
            self.gazebo_wheelBackRight_pub = self.create_publisher(Float64, "wheelBackRight/cmd_vel", 10)

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

    # Define subsystem methods here
    def drive(self, forward_power: float, turning_power: float) -> None:
        """This method drives the robot with the desired forward and turning power."""
        if not self.ready_to_drive:
            self.get_logger().warn("The digger is not raised. Cannot drive.")
            return False

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

        # Send velocity (not duty cycle) motor commands to the motor_control_node
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                can_id=self.FRONT_LEFT_DRIVE, type="velocity", value=leftPower * self.MAX_DRIVETRAIN_RPM
            )
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                can_id=self.BACK_LEFT_DRIVE, type="velocity", value=leftPower * self.MAX_DRIVETRAIN_RPM
            )
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                can_id=self.FRONT_RIGHT_DRIVE, type="velocity", value=rightPower * self.MAX_DRIVETRAIN_RPM
            )
        )
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                can_id=self.BACK_RIGHT_DRIVE, type="velocity", value=rightPower * self.MAX_DRIVETRAIN_RPM
            )
        )

        # Publish the wheel speeds to the gazebo simulation
        if self.GAZEBO_SIMULATION:
            self.gazebo_wheelFrontLeft_pub.publish(Float64(data=leftPower))
            self.gazebo_wheelFrontRight_pub.publish(Float64(data=rightPower))
            self.gazebo_wheelBackLeft_pub.publish(Float64(data=leftPower))
            self.gazebo_wheelBackRight_pub.publish(Float64(data=rightPower))
        return True

    def stop(self) -> None:
        """This method stops the drivetrain."""
        self.drive(0.0, 0.0)

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

    def limit_switch_callback(self, msg):
        """This service determines if the digger is raised or not."""
        self.ready_to_drive = msg.digger_top_limit_switch


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
