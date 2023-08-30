# This node contains the main control flow of our robot code.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2022
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: May 2023

# Import the ROS 2 module
import rclpy
from rclpy.node import Node

# Import ROS 2 formatted message types
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage

# Import Python Modules
import multiprocessing  # Allows us to run tasks in parallel using multiple CPU cores!
import subprocess  # This is for the webcam stream subprocesses
import signal  # Allows us to kill subprocesses
import serial  # Serial communication with the Arduino. Install with: <sudo pip3 install pyserial>
import time  # This is for time.sleep()
import os  # Allows us to kill subprocesses
import re  # Enables using regular expressions

# Import our gamepad button mappings
from .gamepad_constants import *

# GLOBAL VARIABLES #
buttons = [0] * 11  # This is to help with button press detection
# Define the possible states of our robot
states = {
    "Teleop": 0,
    "Autonomous": 1,
}

def update_sharedVar(var, new_value):
    """This function updates a shared variable in a multiprocessing context."""
    with var.get_lock():
        var.value = new_value

def get_target_ip(target: str, default: str = "", logger_fn=print):
    """Return the current IP address of Jonathan's laptop using nmap."""
    try:
        nmap = subprocess.Popen(
            ("nmap", "-sn", "192.168.1.1/24"), stdout=subprocess.PIPE
        )
        grep = subprocess.check_output(("grep", target), stdin=nmap.stdout)
        nmap.wait()
        res = re.sub(r".*\((.*)\).*", r"\g<1>", grep.decode())
        if not res:
            raise Exception("target not found")
        return res
    except:
        logger_fn(f"target not found; defaulting to {default}")
        return default


class MainControlNode(Node):
    def drive(self, drive_power, turn_power):
        """This method publishes a ROS2 message with the desired drive power and turning power."""
        # Create a new ROS2 msg
        drive_power_msg = Twist()
        # Default to 0 power for everything at first
        drive_power_msg.angular.x = 0.0
        drive_power_msg.angular.y = 0.0
        drive_power_msg.angular.z = 0.0
        drive_power_msg.linear.x = 0.0
        drive_power_msg.linear.y = 0.0
        drive_power_msg.linear.z = 0.0
        drive_power_msg.linear.x = drive_power  # Forward power
        drive_power_msg.angular.z = turn_power  # Turning power
        self.drive_power_publisher.publish(drive_power_msg)
        # self.get_logger().info(f'Publishing Angular Power: {drive_power_msg.angular.z}, Linear Power: {drive_power_msg.linear.x}')

    def stop(self):
        """This method stops the drivetrain."""
        self.drive(0.0, 0.0)

    def auto_dig_procedure(self):
        """This method lays out the procedure for autonomously digging!"""
        # Print to the terminal
        print("\nStarting Autonomous Digging Procedure!")
        update_sharedVar(self.sharedVar_diggerToggled, True) # Start the digger

        self.arduino.read_all()  # Read all messages from the serial buffer to clear them out
        time.sleep(2)  # Wait a bit for the drum motor to get up to speed

        # Tell the Arduino to extend the linear actuator
        self.arduino.write(f"e{chr(self.linear_actuator_speed)}".encode("ascii"))
        while True:  # Wait for a confirmation message from the Arduino
            reading = self.arduino.read()
            print(reading)
            if reading == b"f":
                break

        time.sleep(5)  # Wait for 5 seconds

        # Tell the Arduino to retract the linear actuator
        self.arduino.write(f"r{chr(self.linear_actuator_up_speed)}".encode("ascii"))
        while True:  # Wait for a confirmation message from the Arduino
            reading = self.arduino.read()
            print(reading)
            if reading == b"s":
                break

        # Reverse the digging drum
        update_sharedVar(self.sharedVar_diggerToggled, False)
        update_sharedVar(self.sharedVar_reverseDigger, True)

        time.sleep(5)  # Wait for 5 seconds

        # Stop the digger
        update_sharedVar(self.sharedVar_reverseDigger, False)

        print("Autonomous Digging Procedure Complete!\n")
        # Enter teleop mode after this autonomous command is finished
        update_sharedVar(self.state, states["Teleop"])

    def auto_offload_procedure(self):
        """This method lays out the procedure for autonomously offloading!"""
        print("\nStarting Autonomous Offload Procedure!")

        # Search for an Apriltag before continuing
        print("Searching for an Apriltag to dock with...")
        update_sharedVar(self.sharedVar_apriltagX, 0.0)
        # Add a small delay to see if we can see an Apriltag already
        time.sleep(0.05)
        while self.sharedVar_apriltagX.value == 0.0:
            # Turn slowly to look for an Apriltag
            update_sharedVar(self.sharedVar_autoDrivePower, 0.0)
            update_sharedVar(self.sharedVar_autoTurnPower, 0.3)
        print(
            f"Apriltag found! x: {self.sharedVar_apriltagX.value}, z: {self.sharedVar_apriltagZ.value}, yaw :{self.sharedVar_apriltagYaw.value}"
        )
        update_sharedVar(self.sharedVar_autoDrivePower, 0.0)
        update_sharedVar(self.sharedVar_autoTurnPower, 0.0)

        while (
            self.sharedVar_apriltagZ.value >= 1.5
        ):  # Continue correcting until we are within 1.5 meters of the tag #TODO: Tune this distance
            update_sharedVar(self.sharedVar_autoDrivePower, self.autonomous_driving_power)
            # TODO: Tune both of these P constants on the actual robot
            update_sharedVar(self.sharedVar_autoTurnPower, -1 * (0.5 * self.sharedVar_apriltagYaw.value + 0.5 * self.sharedVar_apriltagX.value))
            print(
                f"Tracking Apriltag with pose x: {self.sharedVar_apriltagX.value}, z: {self.sharedVar_apriltagZ.value}, yaw :{self.sharedVar_apriltagYaw.value}"
            )
            # Add a small delay so we don't overload ROS with too many messages
            time.sleep(0.05)
        update_sharedVar(self.sharedVar_autoDrivePower, 0.0)
        update_sharedVar(self.sharedVar_autoTurnPower, 0.0)

        # Finish docking with the trough
        print("Docking with the trough")
        update_sharedVar(self.sharedVar_autoDrivePower, self.autonomous_driving_power)
        update_sharedVar(self.sharedVar_autoTurnPower, 0.0)
        # TODO: Tune this timing (how long to drive straight for at the end of docking)
        time.sleep(4)
        update_sharedVar(self.sharedVar_autoDrivePower, 0.0)
        update_sharedVar(self.sharedVar_autoTurnPower, 0.0)

        print("Commence Offloading!")
        update_sharedVar(self.sharedVar_offloaderToggled, 1) # Start Offloading
        # TODO: Tune this timing (how long to run the offloader for)
        time.sleep(10)
        update_sharedVar(self.sharedVar_offloaderToggled, 0) # Stop Offloading

        # Print to the terminal
        print("Autonomous Offload Procedure Complete!\n")
        # Enter teleop mode after this autonomous command is finished
        update_sharedVar(self.state, states["Teleop"])

    def __init__(self):
        """Initialize the ROS2 Node."""
        super().__init__("rovr_control")
        
        # Define default values for our ROS parameters below #
        
        # Maximum driving/turning speeds
        self.declare_parameter("autonomous_driving_power", 0.25)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("max_drive_power", 1.0)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("max_turn_power", 1.0)  # Measured in Duty Cycle (0.0-1.0)
        # Linear Actuator Speeds
        self.declare_parameter("linear_actuator_speed", 8)  # Duty Cycle value between 0-100 (not 0.0-1.0)
        self.declare_parameter("linear_actuator_up_speed", 40)  # Duty Cycle value between 0-100 (not 0.0-1.0)
        self.declare_parameter("small_linear_actuator_speed", 75)  # Duty Cycle value between 0-100 (not 0.0-1.0)
        # Motor Speeds
        self.declare_parameter("digger_rotation_power", 0.4)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("drum_belt_power", 0.2)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("conveyor_belt_power", 0.35)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("offload_belt_power", 0.35)  # Measured in Duty Cycle (0.0-1.0)
        
        # Assign the ROS Parameters
        self.autonomous_driving_power = self.get_parameter("autonomous_driving_power").value
        self.max_drive_power = self.get_parameter("max_drive_power").value
        self.max_turn_power = self.get_parameter("max_turn_power").value
        self.digger_rotation_power = self.get_parameter("digger_rotation_power").value
        self.drum_belt_power = self.get_parameter("drum_belt_power").value
        self.conveyor_belt_power = self.get_parameter("conveyor_belt_power").value
        self.offload_belt_power = self.get_parameter("offload_belt_power").value
        self.linear_actuator_speed = self.get_parameter("linear_actuator_speed").value
        self.linear_actuator_up_speed = self.get_parameter("linear_actuator_up_speed").value
        self.small_linear_actuator_speed = self.get_parameter("small_linear_actuator_speed").value
        
        print("autonomous_driving_power has been set to:", self.autonomous_driving_power)
        print("max_drive_power has been set to:", self.max_drive_power)
        print("max_turn_power has been set to:", self.max_turn_power)
        print("linear_actuator_speed has been set to:", self.linear_actuator_speed)
        print("linear_actuator_up_speed has been set to:", self.linear_actuator_up_speed)
        print("small_linear_actuator_speed has been set to:", self.small_linear_actuator_speed)
        print("digger_rotation_power has been set to:", self.digger_rotation_power)
        print("drum_belt_power has been set to:", self.drum_belt_power)
        print("conveyor_belt_power has been set to:", self.conveyor_belt_power)
        print("offload_belt_power has been set to:", self.offload_belt_power)

        # NOTE: The code commented out below is for dynamic ip address asignment, but we haven't gotten it to work yet
        # self.target_ip = get_target_ip('blixt-G14', '192.168.1.110', self.get_logger().info)
        # self.get_logger().info(f'set camera stream target ip to {self.target_ip}')

        # Try connecting to the Arduino over Serial
        try:
            # Set this as a static Serial port!
            self.arduino = serial.Serial("/dev/Arduino_Uno", 9600)
        except Exception as e:
            print(e)  # If an exception is raised, print it, and then move on

        # Define shared variables here (shared among all running processes)
        self.sharedVar_state = multiprocessing.Value("i", states["Teleop"])
        self.sharedVar_apriltagX = multiprocessing.Value("f", 0.0)
        self.sharedVar_apriltagZ = multiprocessing.Value("f", 0.0)
        self.sharedVar_apriltagYaw = multiprocessing.Value("f", 0.0)
        self.sharedVar_autoDrivePower = multiprocessing.Value("f", 0.0)
        self.sharedVar_autoTurnPower = multiprocessing.Value("f", 0.0)
        self.sharedVar_diggerToggled = multiprocessing.Value("d", 0)
        self.sharedVar_reverseDigger = multiprocessing.Value("d", 0)
        self.sharedVar_offloaderToggled = multiprocessing.Value("d", 0)
        
        self.digger_extend_toggled = False
        self.camera_view_toggled = False

        # By default these camera streams will not exist yet
        self.front_camera = None
        self.back_camera = None
        # By default these processes will also not exist yet
        self.autonomous_digging_process = None
        self.autonomous_offload_process = None

        self.apriltag_camera_x_offset = 0.1905  # Measured in Meters

        # Actuators Publisher
        self.actuators_publisher = self.create_publisher(String, "cmd_actuators", 10)
        actuators_timer_period = 0.05  # how often to publish measured in seconds
        self.actuators_timer = self.create_timer(
            actuators_timer_period, self.actuators_timer_callback
        )
        # Drive Power Publisher
        self.drive_power_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # Apriltag Pose Publisher
        self.apriltag_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "apriltag_pose", 10
        )

        # Joystick Subscriber
        self.joy_subscription = self.create_subscription(
            Joy, "joy", self.joystick_callback, 10
        )
        # Apriltags Subscriber
        self.apriltags_subscription = self.create_subscription(
            TFMessage, "tf", self.apriltags_callback, 10
        )

    def apriltags_callback(self, msg):
        """Process the Apriltag detections."""
        array = msg.transforms
        entry = array.pop()

        # Create a PoseWithCovarianceStamped object from the Apriltag detection
        pose_object = PoseWithCovarianceStamped()
        pose_object.header = entry.header
        pose_object.pose.pose.position.x = (
            entry.transform.translation.x + self.apriltag_camera_x_offset
        )
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
        update_sharedVar(self.sharedVar_apriltagX, entry.transform.translation.x + self.apriltag_camera_x_offset)
        # Foward-Backward Distance to the tag (measured in meters)
        update_sharedVar(self.sharedVar_apriltagZ, entry.transform.translation.z)
        # Yaw Angle error to the tag's orientation (measured in radians)
        update_sharedVar(self.sharedVar_apriltagYaw, entry.transform.rotation.y)
        # print('x:', self.sharedVar_apriltagX.value, 'z:', self.sharedVar_apriltagZ.value, 'yaw:', self.sharedVar_apriltagYaw.value)

    def publish_actuator_cmd(self, cmd: str):
        """This method publishes the given actuator command to the 'cmd_actuators' topic."""
        msg = String()
        msg.data = cmd
        self.actuators_publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data) # Print to the terminal

    def actuators_timer_callback(self):
        """This method publishes a message detailing what all the actuators should be doing."""
        cmd = ""
        if self.sharedVar_diggerToggled.value:
            cmd += " DIGGER_ON"
        elif self.sharedVar_reverseDigger.value:
            cmd += " sharedVar_reverseDigger"
        else:
            cmd += " DIGGER_OFF"
        if self.sharedVar_offloaderToggled.value:
            cmd += " OFFLOADER_ON"
        elif not self.sharedVar_offloaderToggled.value:
            cmd += " OFFLOADER_OFF"
        self.publish_actuator_cmd(cmd)
        if self.sharedVar_state.value == states["Autonomous"]:
            self.drive(self.sharedVar_autoDrivePower.value, self.sharedVar_autoTurnPower.value)

    def joystick_callback(self, msg):
        """This method is called whenever a joystick message is received."""

        # TELEOP CONTROLS BELOW #
        if self.sharedVar_state.value == states["Teleop"]:
            # Drive the robot using joystick input during Teleop
            drive_power = (
                msg.axes[RIGHT_JOYSTICK_VERTICAL_AXIS] * self.max_drive_power
            )  # Forward power
            turn_power = (
                msg.axes[LEFT_JOYSTICK_HORIZONTAL_AXIS] * self.max_turn_power
            )  # Turning power
            self.drive(drive_power, turn_power)

            # Check if the digger button is pressed
            if msg.buttons[X_BUTTON] == 1 and buttons[X_BUTTON] == 0:
                update_sharedVar(self.sharedVar_diggerToggled, not self.sharedVar_diggerToggled.value)
            # Check if the offloader button is pressed
            if msg.buttons[B_BUTTON] == 1 and buttons[B_BUTTON] == 0:
                update_sharedVar(self.sharedVar_offloaderToggled, not self.sharedVar_offloaderToggled.value)

            # Check if the digger_extend button is pressed
            if msg.buttons[A_BUTTON] == 1 and buttons[A_BUTTON] == 0:
                self.digger_extend_toggled = not self.digger_extend_toggled
                if self.digger_extend_toggled:
                    # Tell the Arduino to extend the linear actuator
                    self.arduino.write(f"e{chr(self.linear_actuator_speed)}".encode("ascii"))
                else:
                    # Tell the Arduino to retract the linear actuator
                    self.arduino.write(
                        f"r{chr(self.linear_actuator_up_speed)}".encode("ascii")
                    )

            # Stop the linear actuator
            if msg.buttons[Y_BUTTON] == 1 and buttons[Y_BUTTON] == 0:
                # Send stop command to the Arduino
                self.arduino.write(f"e{chr(0)}".encode("ascii"))

            if msg.buttons[RIGHT_BUMPER] == 1 and buttons[RIGHT_BUMPER] == 0:
                update_sharedVar(self.sharedVar_reverseDigger, not self.sharedVar_reverseDigger.value)

            # NOTE: This hasn't been tested/used yet
            # # Small linear actuator controls
            # if msg.buttons[RIGHT_BUMPER] == 1 and buttons[RIGHT_BUMPER] == 0:
            #   self.arduino.write(f'a{chr(small_linear_actuator_speed)}'.encode(
            #     'ascii'))  # Extend the small linear actuator
            # if msg.buttons[LEFT_BUMPER] == 1 and buttons[LEFT_BUMPER] == 0:
            #   self.arduino.write(f'b{chr(small_linear_actuator_speed)}'.encode(
            #     'ascii'))  # Retract the small linear actuator

        # THE CONTROLS BELOW ALWAYS WORK #

        # Check if the autonomous digging button is pressed
        if msg.buttons[BACK_BUTTON] == 1 and buttons[BACK_BUTTON] == 0:
            if self.sharedVar_state.value == states["Teleop"]:
                update_sharedVar(self.sharedVar_state, states["Autonomous"])
                self.autonomous_digging_process = multiprocessing.Process(
                    target=self.auto_dig_procedure
                )
                self.autonomous_digging_process.start()  # Start the auto dig process
            elif self.sharedVar_state.value == states["Autonomous"]:
                update_sharedVar(self.sharedVar_state, states["Teleop"])
                self.autonomous_digging_process.kill()  # Kill the auto dig process
                print("Autonomous Digging Procedure Terminated\n")
                # After we finish this autonomous operation, start with the digger off
                update_sharedVar(self.sharedVar_diggerToggled, 0)
                # Stop the linear actuator
                self.arduino.write(f"e{chr(0)}".encode("ascii"))

        # Check if the autonomous offload button is pressed
        if msg.buttons[LEFT_BUMPER] == 1 and buttons[LEFT_BUMPER] == 0:
            if self.sharedVar_state.value == states["Teleop"]:
                update_sharedVar(self.sharedVar_state, states["Autonomous"])
                self.autonomous_offload_process = multiprocessing.Process(
                    target=self.auto_offload_procedure
                )
                self.autonomous_offload_process.start()  # Start the auto dig process
            elif self.sharedVar_state.value == states["Autonomous"]:
                update_sharedVar(self.sharedVar_state, states["Teleop"])
                self.autonomous_offload_process.kill()  # Kill the auto dig process
                print("Autonomous Offload Procedure Terminated\n")
                # After we finish this autonomous operation, start with the offloader off
                update_sharedVar(self.sharedVar_offloaderToggled, 0)
                # Stop driving
                self.stop()

        # Check if the camera toggle button is pressed
        if msg.buttons[START_BUTTON] == 1 and buttons[START_BUTTON] == 0:
            self.camera_view_toggled = not self.camera_view_toggled
            if (
                self.camera_view_toggled
            ):  # Start streaming /dev/front_webcam on port 5000
                if self.back_camera is not None:
                    # Kill the self.back_camera process
                    os.killpg(os.getpgid(self.back_camera.pid), signal.SIGTERM)
                    self.back_camera = None
                # self.get_logger().info(f'using ip {self.target_ip}')
                self.front_camera = subprocess.Popen(
                    'gst-launch-1.0 v4l2src device=/dev/front_webcam ! "video/x-raw,width=640,height=480,framerate=15/1" ! nvvidconv ! "video/x-raw,format=I420" ! x264enc bitrate=300 tune=zerolatency speed-preset=ultrafast ! "video/x-h264,stream-format=byte-stream" ! h264parse ! rtph264pay ! udpsink host=192.168.1.110 port=5000',
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
                    'gst-launch-1.0 v4l2src device=/dev/back_webcam ! "video/x-raw,width=640,height=480,framerate=15/1" ! nvvidconv ! "video/x-raw,format=I420" ! x264enc bitrate=300 tune=zerolatency speed-preset=ultrafast ! "video/x-h264,stream-format=byte-stream" ! h264parse ! rtph264pay ! udpsink host=192.168.1.110  port=5000',
                    shell=True,
                    preexec_fn=os.setsid,
                )

        # Update new button states (this allows us to detect changing button states)
        for index in range(len(buttons)):
            buttons[index] = msg.buttons[index]


def main(args=None):
    """The main function."""
    rclpy.init(args=args)
    print("Hello from the rovr_control package!")

    node = MainControlNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
