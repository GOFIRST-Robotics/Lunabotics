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
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage

# Import Python Modules
import multiprocessing # Allows us to run tasks in parallel using multiple CPU cores!
import subprocess # This is for the webcam stream subprocesses
import signal # Allows us to kill subprocesses
import serial # Serial communication with the Arduino. Install with: <sudo pip3 install pyserial>
import time # This is for time.sleep()
import os # Allows us to kill subprocesses

# Import our gamepad button mappings
from .gamepad_constants import *
# This is to help with button press detection
buttons = [0] * 12

# Define some initial button states
digger_toggled = False
offloader_toggled = False
digger_extend_toggled = False
camera_view_toggled = False

# By default these camera streams will not exist yet
camera0 = None
camera1 = None
# By default these processes will also not exist yet
autonomous_digging_process = None
autonomous_offload_process = None

# Define the possible states of our robot
states = {'Teleop': 0, 'Autonomous': 1, 'Auto_Dig': 2, 'Auto_Offload': 3, 'Emergency_Stop': 4}

# Define the maximum driving power of the robot (measured in duty cycle)
dig_driving_power = 0.5 # The power to drive at while autonomously digging
max_drive_power = 1.0
max_turn_power = 1.0
    
    
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
        drive_power_msg.linear.x = drivePower # Forward power
        drive_power_msg.angular.z = turnPower # Turning power
        self.drive_power_publisher.publish(drive_power_msg)
        #self.get_logger().info(f'Publishing Angular Power: {drive_power_msg.angular.z}, Linear Power: {drive_power_msg.linear.x}')

    # Stop the drivetrain
    def stop(self):
        self.drive(0.0, 0.0)


    # This method lays out the procedure for autonomously digging!
    def auto_dig_procedure(self, state):
        print('\nStarting Autonomous Digging Procedure!') # Print to the terminal
        self.digger(True) # Start the digger
        self.offloader(False) # Stop the offloader if it is currently running

        self.arduino.read_all() # Read all messages from the serial buffer to clear them out
        time.sleep(5) # TODO: Change this to wait for the digger motor to reach a certain speed
        
        self.arduino.write('e'.encode('utf_8')) # Tell the Arduino to extend the linear actuator
        while True: # Wait for a confirmation message from the Arduino
            if self.arduino.read() == 'f'.encode('utf_8'):
                break
        
        self.drive(dig_driving_power, 0.0) # Start driving forward slowly
        time.sleep(20) # TODO: Tune this timing (how long do we want to drive for?)
        self.stop() # Stop the drivetrain
        self.digger(False) # Stop the digger
        
        self.arduino.write('r'.encode('utf_8')) # Tell the Arduino to retract the linear actuator
        while True: # Wait for a confirmation message from the Arduino
            if self.arduino.read() == 's'.encode('utf_8'):
                break
        
        print('Autonomous Digging Procedure Complete!\n') # Print to the terminal
        state.value = states['Teleop'] # Enter teleop mode after this autonomous command is finished


    # This method lays out the procedure for autonomously offloading!
    def auto_offload_procedure(self, state):
        print('\nStarting Autonomous Offload Procedure!') # Print to the terminal
        self.digger(False) # Stop the digger if it is currently running
        self.offloader(False) # Stop the offloader if it is currently running
        
        # TODO: If there is an apriltag continue, else search for one

        # TODO: Once an Apriltag has been found, do the following:
        # Turn so that the apriltag is close to the edge of the field of view
        # Drive forward until it leaves field of view
        # Turn back 
        # Repeat until distance to Apriltag is small

        self.offloader(True) # Start Offloading
        time.sleep(10) # TODO: Tune this timing
        self.offloader(False) # Stop Offloading
        
        print('Autonomous Offload Procedure Complete!\n') # Print to the terminal
        state.value = states['Teleop'] # Enter teleop mode after this autonomous command is finished
            

    # Initialize the ROS2 Node
    def __init__(self):
        super().__init__('rovr_control')
        
        # Try connecting to the Arduino over Serial
        try:
            self.arduino = serial.Serial('/dev/Arduino_Uno', 9600) # Set this as a static Serial port!
        except Exception as e:
            print(e) # If an exception is raised, print it, and then move on
        
        self.manager = multiprocessing.Manager() # This allows us to modify our current state from within autonomous processes
        self.current_state = self.manager.Value('i', states['Teleop']) # Define our robot's initial state

        # Actuators Publisher
        self.actuators_publisher = self.create_publisher(String, 'cmd_actuators', 10)
        actuators_timer_period = 0.05  # how often to publish measured in seconds
        self.actuators_timer = self.create_timer(actuators_timer_period, self.actuators_timer_callback)
        # Drive Power Publisher
        self.drive_power_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Joystick Subscriber
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joystick_callback, 10)
        # Apriltags Subscriber
        self.joy_subscription = self.create_subscription(TFMessage, 'tf', self.apriltags_callback, 10)


    # Process Apriltag Detections
    def apriltags_callback(self, msg):
        apriltag_position_x = msg.____ #TODO something
        apriltag_position_y = msg.____ #TODO something (dont need this?)
        apriltag_position_z = msg.____ #TODO something
        apriltag_orientation_x = msg.____ #TODO something
        apriltag_orientation_y = msg.____ #TODO something (dont need this?)
        apriltag_orientation_z = msg.____ #TODO something (dont need this?)


    # This method publishes the given actuator command to 'cmd_actuators'
    def publish_actuator_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.actuators_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data) # Print to the terminal

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
        # Python is silly and you have to declare global variables like this before using them
        global digger_toggled
        global digger_extend_toggled
        global offloader_toggled

        if self.current_state.value == states['Emergency_Stop']:
            self.publish_actuator_cmd('STOP_ALL_ACTUATORS')
        elif self.current_state.value == states['Teleop']:
            cmd = ''

            if digger_toggled:
                cmd += ' DIGGER_ON'
            elif not digger_toggled:
                cmd += ' DIGGER_OFF'
            if offloader_toggled:
                cmd += ' OFFLOADER_ON'
            elif not offloader_toggled:
                cmd += ' OFFLOADER_OFF'
                
            self.publish_actuator_cmd(cmd)
            
            
    # When a joystick input is recieved, this callback method processes the input accordingly
    def joystick_callback(self, msg):
        
        # Python is silly and you have to declare global variables like this before using them
        global digger_toggled
        global camera_view_toggled
        global digger_extend_toggled
        global offloader_toggled
        global autonomous_digging_process
        global autonomous_offload_process
        global camera0
        global camera1
        
        # TELEOP CONTROLS BELOW #

        if self.current_state.value == states['Teleop']:

            # Drive the robot using joystick input during Teleop
            drivePower = (msg.axes[RIGHT_JOYSTICK_VERTICAL_AXIS]) * max_drive_power # Forward power
            turnPower = (msg.axes[LEFT_JOYSTICK_HORIZONTAL_AXIS]) * max_turn_power # Turning power
            self.drive(drivePower, turnPower)
        
            # Check if the digger button is pressed
            if msg.buttons[X_BUTTON] == 1 and buttons[X_BUTTON] == 0:
                digger_toggled = not digger_toggled
            # Check if the offloader button is pressed
            if msg.buttons[B_BUTTON] == 1 and buttons[B_BUTTON] == 0:
                offloader_toggled = not offloader_toggled
                
            # Check if the digger_extend button is pressed
            if msg.buttons[A_BUTTON] == 1 and buttons[A_BUTTON] == 0:
                digger_extend_toggled = not digger_extend_toggled
                if digger_extend_toggled:
                    self.arduino.write('e'.encode('utf_8')) # Tell the Arduino to extend the linear actuator
                else:
                    self.arduino.write('r'.encode('utf_8')) # Tell the Arduino to retract the linear actuator

        # THE CONTROLS BELOW ALWAYS WORK #

        # Check if the autonomous digging button is pressed
        if msg.buttons[Y_BUTTON] == 1 and buttons[Y_BUTTON] == 0:
            if self.current_state.value == states['Teleop']:
                self.current_state.value = states['Auto_Dig']
                autonomous_digging_process = multiprocessing.Process(target=self.auto_dig_procedure, args=[self.current_state])
                autonomous_digging_process.start() # Start the auto dig process
            elif self.current_state.value == states['Auto_Dig']:
                self.current_state.value = states['Teleop']
                autonomous_digging_process.kill() # Kill the auto dig process
                print('Autonomous Digging Procedure Terminated\n')
                digger_toggled = False # When we enter teleop mode, start with the digger off
                offloader_toggled = False # When we enter teleop mode, start with the offloader off
        # Check if the autonomous offload button is pressed
        if msg.buttons[BACK_BUTTON] == 1 and buttons[BACK_BUTTON] == 0:
            if self.current_state.value == states['Teleop']:
                self.current_state.value = states['Auto_Offload']
                autonomous_offload_process = multiprocessing.Process(target=self.auto_offload_procedure, args=[self.current_state])
                autonomous_offload_process.start() # Start the auto dig process
            elif self.current_state.value == states['Auto_Offload']:
                self.current_state.value = states['Teleop']
                autonomous_offload_process.kill() # Kill the auto dig process
                print('Autonomous Offload Procedure Terminated\n')
                digger_toggled = False # When we enter teleop mode, start with the digger off
                offloader_toggled = False # When we enter teleop mode, start with the offloader off
                
                
        # Check if the camera toggle button is pressed
        if msg.buttons[START_BUTTON] == 1 and buttons[START_BUTTON] == 0:
            camera_view_toggled = not camera_view_toggled
            if camera_view_toggled: # Start streaming /dev/video0 on port 5000
                if camera1 is not None:
                    os.killpg(os.getpgid(camera1.pid), signal.SIGTERM) # Kill the camera1 process
                    camera1 = None
                camera0 = subprocess.Popen('gst-launch-1.0 v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=30/1" ! nvvidconv ! "video/x-raw(memory:NVMM),format=I420" ! omxh265enc bitrate=200000 ! "video/x-h265,stream-format=byte-stream" ! h265parse ! rtph265pay ! udpsink host=192.168.1.40 port=5000', shell=True, preexec_fn=os.setsid)
            else: # Start streaming /dev/video1 on port 5000
                if camera0 is not None:
                    os.killpg(os.getpgid(camera0.pid), signal.SIGTERM) # Kill the camera0 process
                    camera0 = None
                camera1 = subprocess.Popen('gst-launch-1.0 v4l2src device=/dev/video1 ! "video/x-raw,width=640,height=480,framerate=30/1" ! nvvidconv ! "video/x-raw(memory:NVMM),format=I420" ! omxh265enc bitrate=200000 ! "video/x-h265,stream-format=byte-stream" ! h265parse ! rtph265pay ! udpsink host=192.168.1.40 port=5000', shell=True, preexec_fn=os.setsid)

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
