# This node contains the main control flow of our robot code.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2022
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2023

# Import the ROS 2 module
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor  # This is needed to run multiple callbacks in a single thread

# Import ROS 2 formatted message types
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import SetPower
from rovr_interfaces.srv import Stop, Drive, MotorCommandGet

# Import Python Modules
import asyncio  # Allows the creation of asynchronous autonomous procedures!
import subprocess  # This is for the webcam stream subprocesses
import signal  # Allows us to kill subprocesses
import os  # Allows us to kill subprocesses

# Import our logitech gamepad button mappings
from .gamepad_constants import *

# Uncomment the line below to use the Xbox controller mappings instead
# from .xbox_controller_constants import *

# GLOBAL VARIABLES #
buttons = [0] * 11  # This is to help with button press detection
# Define the possible states of our robot
states = {"Teleop": 0, "Auto_Dig": 1, "Auto_Offload": 2}


class MainControlNode(Node):
    def __init__(self) -> None:
        """Initialize the ROS2 Node."""
        super().__init__("rovr_control")

        # Define default values for our ROS parameters below #
        self.declare_parameter("autonomous_driving_power", 0.25)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("max_drive_power", 1.0)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("max_turn_power", 1.0)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("conveyor_belt_power", 0.35)  # Measured in Duty Cycle (0.0-1.0)

        # Assign the ROS Parameters to member variables below #
        self.autonomous_driving_power = self.get_parameter("autonomous_driving_power").value
        self.max_drive_power = self.get_parameter("max_drive_power").value
        self.max_turn_power = self.get_parameter("max_turn_power").value
        self.conveyor_belt_power = self.get_parameter("conveyor_belt_power").value

        # Print the ROS Parameters to the terminal below #
        print("autonomous_driving_power has been set to:", self.autonomous_driving_power)
        print("max_drive_power has been set to:", self.max_drive_power)
        print("max_turn_power has been set to:", self.max_turn_power)
        print("conveyor_belt_power has been set to:", self.conveyor_belt_power)

        # Define some initial states here
        self.state = states["Teleop"]
        self.camera_view_toggled = False
        self.front_camera = None
        self.back_camera = None
        self.autonomous_digging_process = None
        self.autonomous_offload_process = None

        # This is a hard-coded physical constant (how far off-center the apriltag camera is)
        self.apriltag_camera_offset = 0.1905  # Measured in Meters

        # These variables store the most recent Apriltag pose
        self.apriltagX = 0.0
        self.apriltagZ = 0.0
        self.apriltagYaw = 0.0

        # Define service clients here
        self.cli_conveyor_toggle = self.create_client(SetPower, "conveyor/toggle")
        self.cli_conveyor_stop = self.create_client(Stop, "conveyor/stop")
        self.cli_conveyor_setPower = self.create_client(SetPower, "conveyor/setPower")
        self.cli_drivetrain_stop = self.create_client(Stop, "drivetrain/stop")
        self.cli_drivetrain_drive = self.create_client(Drive, "drivetrain/drive")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define publishers and subscribers here
        self.drive_power_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.apriltag_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, "apriltag_pose", 10)
        self.joy_subscription = self.create_subscription(Joy, "joy", self.joystick_callback, 10)
        self.apriltags_subscription = self.create_subscription(TFMessage, "tf", self.apriltags_callback, 10)

    def stop_all_subsystems(self) -> None:
        """This method stops all subsystems on the robot."""
        self.cli_conveyor_stop.call_async(Stop.Request())  # Stop the conveyor
        self.cli_drivetrain_stop.call_async(Stop.Request())  # Stop the drivetrain
        # TODO: Stop the conveyor pulley system (height adjust) using the new service

    def end_autonomous(self) -> None:
        """This method returns to teleop control."""
        self.stop_all_subsystems()  # Stop all subsystems
        self.state = states["Teleop"]  # Return to Teleop mode

    async def auto_dig_procedure(self) -> None:
        """This method lays out the procedure for autonomously digging!"""
        print("\nStarting Autonomous Digging Procedure!")
        try:  # Wrap the autonomous procedure in a try-except
            await self.cli_conveyor_setPower.call_async(SetPower.Request(conveyor_belt_power=self.conveyor_belt_power))
            # TODO: Lower the conveyor into the ground
            # TODO: Wait for the goal height to be reached (wait for a True message on /conveyor/goal_reached)
            # Start driving forward
            await self.cli_drivetrain_drive.call_async(Drive.Request(forward_power=self.autonomous_driving_power, turning_power=0.0))
            # TODO: Drive forward until our conveyor is full OR we reach the end of the arena OR we reach an obstacle
            await self.cli_drivetrain_stop.call_async(Stop.Request())
            await self.cli_conveyor_stop.call_async(Stop.Request())
            # TODO: Raise the conveyor back up a bit
            # TODO: Wait for the goal height to be reached (wait for a True message on /conveyor/goal_reached)
            print("Autonomous Digging Procedure Complete!\n")
            self.end_autonomous()  # Return to Teleop mode
        except asyncio.CancelledError:  # Put termination code here
            print("Autonomous Digging Procedure Terminated\n")
            self.end_autonomous()  # Return to Teleop mode

    async def auto_offload_procedure(self) -> None:
        """This method lays out the procedure for autonomously offloading!"""
        print("\nStarting Autonomous Offload Procedure!")
        try:  # Wrap the autonomous procedure in a try-except
            # TODO: Send the robot to the proper offloading coordinates using SLAM
            await self.cli_drivetrain_stop.call_async(Stop.Request())  # Stop the drivetrain
            # TODO: Raise up the conveyor in preparation for dumping
            # TODO: Wait for the goal height to be reached (wait for a True message on /conveyor/goal_reached)
            print("Commence Offloading!")
            await self.cli_conveyor_setPower.call_async(SetPower.Request(conveyor_belt_power=self.conveyor_belt_power))
            await asyncio.sleep(10)  # How long to offload for
            await self.cli_conveyor_stop.call_async(Stop.Request())  # Stop the conveyor belt
            print("Autonomous Offload Procedure Complete!\n")
            self.end_autonomous()  # Return to Teleop mode
        except asyncio.CancelledError:  # Put termination code here
            print("Autonomous Offload Procedure Terminated\n")
            self.end_autonomous()  # Return to Teleop mode

    def apriltags_callback(self, msg: TFMessage) -> None:
        """Process the Apriltag detections."""
        array = msg.transforms
        entry = array.pop()

        # Create a PoseWithCovarianceStamped object from the Apriltag detection
        pose_object = PoseWithCovarianceStamped()
        pose_object.header = entry.header
        pose_object.pose.pose.position.x = entry.transform.translation.x + self.apriltag_camera_offset
        pose_object.pose.pose.position.y = entry.transform.translation.y
        pose_object.pose.pose.position.z = entry.transform.translation.z
        pose_object.pose.pose.orientation.x = entry.transform.rotation.x
        pose_object.pose.pose.orientation.y = entry.transform.rotation.y
        pose_object.pose.pose.orientation.z = entry.transform.rotation.z
        pose_object.pose.pose.orientation.w = entry.transform.rotation.w
        pose_object.pose.covariance = [0.0] * 36
        self.apriltag_pose_publisher.publish(pose_object)

        ## Set the value of these variables used for docking with an Apriltag ##

        # Left-Right Distance to the tag (measured in meters)
        self.apriltagX = entry.transform.translation.x + self.apriltag_camera_offset
        # Forward-Backward Distance to the tag (measured in meters)
        self.apriltagZ = entry.transform.translation.z
        # Yaw Angle error to the tag's orientation (measured in radians)
        self.apriltagYaw = entry.transform.rotation.y
        # print('x:', self.apriltagX, 'z:', self.apriltagZ, 'yaw:', self.apriltagYaw)

    def joystick_callback(self, msg: Joy) -> None:
        """This method is called whenever a joystick message is received."""

        # PUT TELEOP CONTROLS BELOW #

        if self.state == states["Teleop"]:
            # Drive the robot using joystick input during Teleop
            drive_power = msg.axes[RIGHT_JOYSTICK_VERTICAL_AXIS] * self.max_drive_power  # Forward power
            turn_power = msg.axes[LEFT_JOYSTICK_HORIZONTAL_AXIS] * self.max_turn_power  # Turning power
            self.drive_power_publisher.publish(Twist(linear=Vector3(x=drive_power), angular=Vector3(z=turn_power)))

            # Check if the conveyor button is pressed #
            if msg.buttons[X_BUTTON] == 1 and buttons[X_BUTTON] == 0:
                self.cli_conveyor_toggle.call_async(SetPower.Request(conveyor_belt_power=self.conveyor_belt_power))

            # TODO: Manually adjust the height of the conveyor with the left and right triggers

        # THE CONTROLS BELOW ALWAYS WORK #

        # Check if the autonomous digging button is pressed
        if msg.buttons[BACK_BUTTON] == 1 and buttons[BACK_BUTTON] == 0:
            if self.state == states["Teleop"]:
                self.stop_all_subsystems()  # Stop all subsystems
                self.state = states["Auto_Dig"]
                self.autonomous_digging_process = asyncio.ensure_future(
                    self.auto_dig_procedure()
                )  # Start the auto dig process
            elif self.state == states["Auto_Dig"]:
                self.autonomous_digging_process.cancel()  # Terminate the auto dig process

        # Check if the autonomous offload button is pressed
        if msg.buttons[LEFT_BUMPER] == 1 and buttons[LEFT_BUMPER] == 0:
            if self.state == states["Teleop"]:
                self.stop_all_subsystems()  # Stop all subsystems
                self.state = states["Auto_Offload"]
                self.autonomous_offload_process = asyncio.ensure_future(
                    self.auto_offload_procedure()
                )  # Start the auto dig process
            elif self.state == states["Auto_Offload"]:
                self.autonomous_offload_process.cancel()  # Terminate the auto offload process

        # Check if the camera toggle button is pressed
        if msg.buttons[START_BUTTON] == 1 and buttons[START_BUTTON] == 0:
            self.camera_view_toggled = not self.camera_view_toggled
            if self.camera_view_toggled:  # Start streaming /dev/front_webcam on port 5000
                if self.back_camera is not None:
                    # Kill the self.back_camera process
                    os.killpg(os.getpgid(self.back_camera.pid), signal.SIGTERM)
                    self.back_camera = None
                # self.get_logger().info(f'using ip {self.target_ip}')
                self.front_camera = subprocess.Popen(
                    'gst-launch-1.0 v4l2src device=/dev/front_webcam ! "video/x-raw,width=640,height=480,framerate=15/1" ! nvvidconv ! "video/x-raw(memory:NVMM),format=NV12" ! nvv4l2av1enc bitrate=200000 ! "video/x-av1" ! udpsink host=10.133.232.197 port=5000',
                    shell=True,
                    preexec_fn=os.setsid,
                )
            else:  # Start streaming /dev/back_webcam on port 5000
                if self.front_camera is not None:
                    # Kill the self.front_camera process
                    os.killpg(os.getpgid(self.front_camera.pid), signal.SIGTERM)
                    self.front_camera = None
                # self.get_logger().info(f'using ip {self.target_ip}')
                self.back_camera = subprocess.Popen(
                    'gst-launch-1.0 v4l2src device=/dev/back_webcam ! "video/x-raw,width=640,height=480,framerate=15/1" ! nvvidconv ! "video/x-raw(memory:NVMM),format=NV12" ! nvv4l2av1enc bitrate=200000 ! "video/x-av1" ! udpsink host=10.133.232.197  port=5000',
                    shell=True,
                    preexec_fn=os.setsid,
                )

        # Update button states (this allows us to detect changing button states)
        for index in range(len(buttons)):
            buttons[index] = msg.buttons[index]


async def spin(executor: SingleThreadedExecutor) -> None:
    """This function is called in the main function to run the executor."""
    while rclpy.ok():  # While ROS is still running
        executor.spin_once()  # Spin the executor once
        await asyncio.sleep(0)  # Setting the delay to 0 provides an optimized path to allow other tasks to run.


def main(args=None) -> None:
    rclpy.init(args=args)
    print("Hello from the rovr_control package!")

    node = MainControlNode()  # Instantiate the node
    executor = SingleThreadedExecutor()  # Create an executor
    executor.add_node(node)  # Add the node to the executor

    loop = asyncio.get_event_loop()  # Get the event loop
    loop.run_until_complete(spin(executor))  # Run the spin function in the event loop

    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
