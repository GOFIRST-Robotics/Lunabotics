# This node contains the main control flow of our robot code.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2022
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: May 2023

# Import the ROS 2 module
import rclpy
from rclpy.node import Node

# Import ROS 2 formatted message types
from std_msgs.msg import String
from std_msgs.msg import Float32
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

# Import our gamepad button mappings
from .gamepad_constants import *

# GLOBAL VARIABLES #
buttons = [0] * 11  # This is to help with button press detection
# Define the possible states of our robot
states = {'Teleop': 0, 'Autonomous': 1, 'Auto_Dig': 2,
          'Auto_Offload': 3, 'Emergency_Stop': 4}
# Define the maximum driving power of the robot (measured in duty cycle)
# The power to drive at while autonomously digging/offloading
autonomous_driving_power = 0.25
max_drive_power = 1.0
max_turn_power = 1.0

linear_actuator_speed = 10  # Value between 0-100
small_linear_actuator_speed = 100  # Value between 0-100


class MainControlNode(Node):

  # Publish a ROS2 message with the desired drive power and turning power
  def drive(self, drivePower, turnPower):
    # Create a new ROS2 msg
    drive_power_msg = Twist()
    # Default to 0 power for everything at first
    drive_power_msg.angular.x = 0.0
    drive_power_msg.angular.y = 0.0
    drive_power_msg.angular.z = 0.0
    drive_power_msg.linear.x = 0.0
    drive_power_msg.linear.y = 0.0
    drive_power_msg.linear.z = 0.0
    drive_power_msg.linear.x = drivePower  # Forward power
    drive_power_msg.angular.z = turnPower  # Turning power
    self.drive_power_publisher.publish(drive_power_msg)
    # self.get_logger().info(f'Publishing Angular Power: {drive_power_msg.angular.z}, Linear Power: {drive_power_msg.linear.x}')

  # Stop the drivetrain
  def stop(self):
    self.drive(0.0, 0.0)

  # This method lays out the procedure for autonomously digging!

  def auto_dig_procedure(self, state, digger_RPM):
    print('\nStarting Autonomous Digging Procedure!')  # Print to the terminal
    self.digger(True)  # Start the digger
    self.offloader(False)  # Stop the offloader if it is currently running

    self.arduino.read_all()  # Read all messages from the serial buffer to clear them out
    time.sleep(2)  # Wait a bit for the drum motor to get up to speed

    # Tell the Arduino to extend the linear actuator
    self.arduino.write(f'e{chr(linear_actuator_speed)}'.encode('ascii'))
    while True:  # Wait for a confirmation message from the Arduino
      if self.arduino.read() == 'f'.encode('ascii'):
        break

    # Tell the Arduino to retract the linear actuator
    self.arduino.write(f'r{chr(linear_actuator_speed)}'.encode('ascii'))
    while True:  # Wait for a confirmation message from the Arduino
      if self.arduino.read() == 's'.encode('ascii'):
        break
      
    self.digger(False)  # Stop the digger

    print('Autonomous Digging Procedure Complete!\n')  # Print to the terminal
    # Enter teleop mode after this autonomous command is finished
    state.value = states['Teleop']

  # This method lays out the procedure for autonomously offloading!
  def auto_offload_procedure(self, state, apriltag_x, apriltag_z, apriltag_yaw):
    print('\nStarting Autonomous Offload Procedure!')  # Print to the terminal
    self.digger(False)  # Stop the digger if it is currently running
    self.offloader(False)  # Stop the offloader if it is currently running

    # Search for an Apriltag before continuing
    print('Searching for an Apriltag to dock with...')
    apriltag_x.value = 0.0
    # Add a small delay to see if we can see an Apriltag already
    time.sleep(0.05)
    while apriltag_x.value == 0.0:
      self.drive(0.0, 0.3)  # Turn slowly to look for an Apriltag
    print(
      f'Apriltag found! x: {apriltag_x.value}, z: {apriltag_z.value}, yaw :{apriltag_yaw.value}')
    self.stop()

    while apriltag_z.value >= 1:  # Continue correcting until we are within 1 meter of the tag #TODO: Tune this distance
      # TODO: Tune both of these P constants on the actual robot
      self.drive(autonomous_driving_power, 0.5 *
                 apriltag_yaw.value + 0.5 * apriltag_x.value)
      print(
        f'Tracking Apriltag with pose x: {apriltag_x.value}, z: {apriltag_z.value}, yaw :{apriltag_yaw.value}')
      # Add a small delay so we don't overload ROS with too many messages
      time.sleep(0.05)
    self.stop()

    # Finish docking with the trough
    print("Docking with the trough")
    self.drive(0.3, 0.0)
    # TODO: Tune this timing (how long to drive straight for at the end of docking)
    time.sleep(4)
    self.stop()

    print("Commence Offloading!")
    self.offloader(True)  # Start Offloading
    time.sleep(10)  # TODO: Tune this timing (how long to run the offloader for)
    self.offloader(False)  # Stop Offloading

    print('Autonomous Offload Procedure Complete!\n')  # Print to the terminal
    # Enter teleop mode after this autonomous command is finished
    state.value = states['Teleop']

  # Initialize the ROS2 Node

  def __init__(self):
    super().__init__('rovr_control')
    
    self.declare_parameter('ip', '192.168.1.117') # Default Value

    # Try connecting to the Arduino over Serial
    try:
      # Set this as a static Serial port!
      self.arduino = serial.Serial('/dev/Arduino_Uno', 9600)
    except Exception as e:
      print(e)  # If an exception is raised, print it, and then move on

    # This allows us to modify our current state from within autonomous processes
    self.manager = multiprocessing.Manager()
    self.current_state = self.manager.Value(
      'i', states['Teleop'])  # Define our robot's initial state
    self.apriltag_x = self.manager.Value('f', 0.0)
    self.apriltag_z = self.manager.Value('f', 0.0)
    self.apriltag_yaw = self.manager.Value('f', 0.0)
    self.current_digger_RPM = self.manager.Value('f', 0.0)

    # Define some initial button states
    self.digger_toggled = False
    self.offloader_toggled = False
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
    self.actuators_publisher = self.create_publisher(
      String, 'cmd_actuators', 10)
    actuators_timer_period = 0.05  # how often to publish measured in seconds
    self.actuators_timer = self.create_timer(
      actuators_timer_period, self.actuators_timer_callback)
    # Drive Power Publisher
    self.drive_power_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    # Apriltag Pose Publisher
    self.apriltag_pose_publisher = self.create_publisher(
      PoseWithCovarianceStamped, 'apriltag_pose', 10)

    # Joystick Subscriber
    self.joy_subscription = self.create_subscription(
      Joy, 'joy', self.joystick_callback, 10)
    # Apriltags Subscriber
    self.apriltags_subscription = self.create_subscription(
      TFMessage, 'tf', self.apriltags_callback, 10)
    # Digger RPM Subsciber
    self.digger_RPM_subscription = self.create_subscription(
      Float32, 'digger_RPM', self.digger_RPM_callback, 10)

  # Process Apriltag Detections

  def apriltags_callback(self, msg):
    array = msg.transforms
    entry = array.pop()

    # Create a PoseWithCovarianceStamped object from the Apriltag detection
    pose_object = PoseWithCovarianceStamped()
    pose_object.header = entry.header
    pose_object.pose.pose.position.x = entry.transform.translation.x + \
        self.apriltag_camera_x_offset
    pose_object.pose.pose.position.y = entry.transform.translation.y
    pose_object.pose.pose.position.z = entry.transform.translation.z
    pose_object.pose.pose.orientation.x = entry.transform.rotation.x
    pose_object.pose.pose.orientation.y = entry.transform.rotation.y
    pose_object.pose.pose.orientation.z = entry.transform.rotation.z
    pose_object.pose.pose.orientation.w = entry.transform.rotation.w
    pose_object.pose.covariance = [0.0] * 36
    self.apriltag_pose_publisher.publish(pose_object)

    # Set the value of these variables used for docking with an Apriltag
    self.apriltag_x.value = entry.transform.translation.x + \
        self.apriltag_camera_x_offset  # Left-Right Distance to the tag (measured in meters)
    # Foward-Backward Distance to the tag (measured in meters)
    self.apriltag_z.value = entry.transform.translation.z
    # Yaw Angle error to the tag's orientation (measured in radians)
    self.apriltag_yaw.value = entry.transform.rotation.y

    # print('x:', self.apriltag_x.value, 'z:', self.apriltag_z.value, 'yaw:', self.apriltag_yaw.value)

  # Update our variable storing the current digger speed

  def digger_RPM_callback(self, msg):
    self.current_digger_RPM.value = msg.data  # Measured in RPM

  # This method publishes the given actuator command to 'cmd_actuators'

  def publish_actuator_cmd(self, cmd):
    msg = String()
    msg.data = cmd
    self.actuators_publisher.publish(msg)
    # self.get_logger().info('Publishing: "%s"' % msg.data) # Print to the terminal

  # Turns the digger on or off
  def digger(self, on):
    if on:
      self.publish_actuator_cmd("DIGGER_ON")
    else:
      self.publish_actuator_cmd("DIGGER_OFF")
  # Turns the offloader on or off

  def offloader(self, on):
    if on:
      self.publish_actuator_cmd("OFFLOADER_ON")
    else:
      self.publish_actuator_cmd("OFFLOADER_OFF")

  # Publish a message detailing what all the actuators should be doing
  def actuators_timer_callback(self):
    if self.current_state.value == states['Emergency_Stop']:
      self.publish_actuator_cmd('STOP_ALL_ACTUATORS')
    elif self.current_state.value == states['Teleop']:
      cmd = ''
      if self.digger_toggled:
        cmd += ' DIGGER_ON'
      elif not self.digger_toggled:
        cmd += ' DIGGER_OFF'
      if self.offloader_toggled:
        cmd += ' OFFLOADER_ON'
      elif not self.offloader_toggled:
        cmd += ' OFFLOADER_OFF'
      self.publish_actuator_cmd(cmd)

  # When a joystick input is recieved, this callback method processes the input accordingly

  def joystick_callback(self, msg):
    
    ip_param = self.get_parameter('ip').get_parameter_value().string_value

    # TELEOP CONTROLS BELOW #

    if self.current_state.value == states['Teleop']:

      # Drive the robot using joystick input during Teleop
      drivePower = (msg.axes[RIGHT_JOYSTICK_VERTICAL_AXIS]
                    ) * max_drive_power  # Forward power
      turnPower = (msg.axes[LEFT_JOYSTICK_HORIZONTAL_AXIS]
                   ) * max_turn_power  # Turning power
      self.drive(drivePower, turnPower)

      # Check if the digger button is pressed
      if msg.buttons[X_BUTTON] == 1 and buttons[X_BUTTON] == 0:
        self.digger_toggled = not self.digger_toggled
      # Check if the offloader button is pressed
      if msg.buttons[B_BUTTON] == 1 and buttons[B_BUTTON] == 0:
        self.offloader_toggled = not self.offloader_toggled

      # Check if the digger_extend button is pressed
      if msg.buttons[A_BUTTON] == 1 and buttons[A_BUTTON] == 0:
        self.digger_extend_toggled = not self.digger_extend_toggled
        if self.digger_extend_toggled:
          # Tell the Arduino to extend the linear actuator
          self.arduino.write(f'e{chr(linear_actuator_speed)}'.encode('ascii'))
        else:
          # Tell the Arduino to retract the linear actuator
          self.arduino.write(f'r{chr(linear_actuator_speed)}'.encode('ascii'))
          
      # Stop the linear actuator
      if msg.buttons[Y_BUTTON] == 1 and buttons[Y_BUTTON] == 0:
        self.arduino.write(f'e{chr(0)}'.encode('ascii')) # Send stop command to the Arduino
          
      # Small linear actuator controls
      if msg.buttons[RIGHT_BUMPER] == 1 and buttons[RIGHT_BUMPER] == 0:
        self.arduino.write(f'a{chr(small_linear_actuator_speed)}'.encode(
          'ascii'))  # Extend the small linear actuator
      if msg.buttons[LEFT_BUMPER] == 1 and buttons[LEFT_BUMPER] == 0:
        self.arduino.write(f'b{chr(small_linear_actuator_speed)}'.encode(
          'ascii'))  # Retract the small linear actuator

    # THE CONTROLS BELOW ALWAYS WORK #

    # Check if the autonomous digging button is pressed
    if msg.buttons[BACK_BUTTON] == 1 and buttons[BACK_BUTTON] == 0:
      if self.current_state.value == states['Teleop']:
        self.current_state.value = states['Auto_Dig']
        self.autonomous_digging_process = multiprocessing.Process(
          target=self.auto_dig_procedure, args=[self.current_state, self.current_digger_RPM])
        self.autonomous_digging_process.start()  # Start the auto dig process
        # After we finish this autonomous operation, start with the digger off
        self.digger_toggled = False
        # After we finish this autonomous operation, start with the offloader off
        self.offloader_toggled = False
      elif self.current_state.value == states['Auto_Dig']:
        self.current_state.value = states['Teleop']
        self.autonomous_digging_process.kill()  # Kill the auto dig process
        print('Autonomous Digging Procedure Terminated\n')
        
    # Check if the autonomous offload button is pressed
    # if msg.buttons[BACK_BUTTON] == 1 and buttons[BACK_BUTTON] == 0:
    #   if self.current_state.value == states['Teleop']:
    #     self.current_state.value = states['Auto_Offload']
    #     self.autonomous_offload_process = multiprocessing.Process(target=self.auto_offload_procedure, args=[
    #                                                               self.current_state, self.apriltag_x, self.apriltag_z, self.apriltag_yaw])
    #     self.autonomous_offload_process.start()  # Start the auto dig process
    #     # After we finish this autonomous operation, start with the digger off
    #     self.digger_toggled = False
    #     # After we finish this autonomous operation, start with the offloader off
    #     self.offloader_toggled = False
    #   elif self.current_state.value == states['Auto_Offload']:
    #     self.current_state.value = states['Teleop']
    #     self.autonomous_offload_process.kill()  # Kill the auto dig process
    #     print('Autonomous Offload Procedure Terminated\n')

    # Check if the camera toggle button is pressed
    if msg.buttons[START_BUTTON] == 1 and buttons[START_BUTTON] == 0:
      self.camera_view_toggled = not self.camera_view_toggled
      if self.camera_view_toggled:  # Start streaming /dev/front_webcam on port 5000
        if self.back_camera is not None:
          # Kill the self.back_camera process
          os.killpg(os.getpgid(self.back_camera.pid), signal.SIGTERM)
          self.back_camera = None
        self.front_camera = subprocess.Popen(
          f'gst-launch-1.0 v4l2src device=/dev/front_webcam ! "video/x-raw,width=640,height=480,framerate=15/1" ! nvvidconv ! "video/x-raw,format=I420" ! x264enc bitrate=300 tune=zerolatency speed-preset=ultrafast ! "video/x-h264,stream-format=byte-stream" ! h264parse ! rtph264pay ! udpsink host={ip_param} port=5000', shell=True, preexec_fn=os.setsid)
      else:  # Start streaming /dev/back_webcam on port 5000
        if self.front_camera is not None:
          # Kill the self.front_camera process
          os.killpg(os.getpgid(self.front_camera.pid), signal.SIGTERM)
          self.front_camera = None
        self.back_camera = subprocess.Popen(
          f'gst-launch-1.0 v4l2src device=/dev/back_webcam ! "video/x-raw,width=640,height=480,framerate=15/1" ! nvvidconv ! "video/x-raw,format=I420" ! x264enc bitrate=300 tune=zerolatency speed-preset=ultrafast ! "video/x-h264,stream-format=byte-stream" ! h264parse ! rtph264pay ! udpsink host={ip_param} port=5000', shell=True, preexec_fn=os.setsid)

    # Update new button states (this allows us to detect changing button states)
    for index in range(len(buttons)):
      buttons[index] = msg.buttons[index]


def main(args=None):
  rclpy.init(args=args)
  print('Hello from the rovr_control package!')

  node = MainControlNode()
  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == '__main__':
  main()
