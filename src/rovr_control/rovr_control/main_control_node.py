# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2022
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: February 2023

# Import ROS 2 modules
import rclpy
from rclpy.node import Node

# Import ROS 2 formatted message types
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy

import multiprocessing # Allows us to run tasks in parallel using multiple CPU cores!
import subprocess # This is for the webcam stream subprocesses
import signal # Allows us to kill subprocesses
import serial # Serial communication with the Arduino. Install with: "sudo pip3 install pyserial"
import time # This is for time.sleep()
import os # Allows us to kill subprocesses

# Import our gamepad button mappings
from .gamepad_constants import *
# This is to help with button press detection
buttons = [0] * 12

dig_button_toggled = False
offload_button_toggled = False
digger_extend_button_toggled = False
camera_view_toggled = False

camera0 = None
camera1 = None

# Define the possible states of our robot
states = {'Teleop': 0, 'Autonomous': 1, 'Auto_Dig': 2, 'Emergency_Stop': 3}

# Define the maximum driving power of the robot (duty cycle)
dig_driving_power = 0.5 # The power to drive at when autonomously digging
max_drive_power = 1.0
max_turn_power = 1.0

# These global values are updated by joystick input
current_drive_power = 0.0
current_turn_power = 0.0
    
# Define a global counter for printing to the terminal less often
counter = 0
    
class MainControlNode(Node):
    
    # This method lays out the procedure for autonomously digging!
    def auto_dig_procedure(self, state, auto_driving):
        self.get_logger().info('Starting Autonomous Digging Procedure!') # Print to the terminal
        time.sleep(5) # TODO: Tune this timing (wait for the digger to get up to speed)
        
        self.arduino.write(1) # Tell the Arduino to extend the linear actuator
        while True: # Wait for a confirmation message from the Arduino
            if self.arduino.read() == 3:
                break
        
        auto_driving.value = True # Start driving forward slowly
        time.sleep(15) # TODO: Tune this timing (how long do we want to drive for?)
        auto_driving.value = False # Stop the drivetrain
        
        self.arduino.write(0) # Tell the Arduino to retract the linear actuator
        while True: # Wait for a confirmation message from the Arduino
            if self.arduino.read() == 4:
                break
        
        self.get_logger().info('Autonomous Digging Procedure Complete!') # Print to the terminal
        state.value = states['Teleop'] # Enter teleop mode after this autonomous command is finished
        

    def __init__(self):
        super().__init__('publisher')
        
        # Try connecting to the Arduino over Serial
        self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=10) # TODO: Is this Serial port correct? I believe the navX will be /dev/ttyACM0
        
        self.manager = multiprocessing.Manager()
        self.current_state = self.manager.Value("i", states['Teleop']) # Define our robot's initial state
        self.auto_driving = self.manager.Value("i", False)

        # Actuators Publisher
        self.actuators_publisher = self.create_publisher(String, 'cmd_actuators', 10)
        actuators_timer_period = 0.05  # how often to publish measured in seconds
        self.actuators_timer = self.create_timer(actuators_timer_period, self.actuators_timer_callback)
        
        # Drive Power Publisher
        self.drive_power_publisher = self.create_publisher(Twist, 'drive_power', 10)
        drive_power_timer_period = 0.05  # how often to publish measured in seconds
        self.drive_power_timer = self.create_timer(drive_power_timer_period, self.drive_power_timer_callback)

        # Joystick Subscriber
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joystick_callback, 10)
        
        # Create our autonomous digging thread
        self.autonomous_digging_process = multiprocessing.Process(target=self.auto_dig_procedure, args=[self.current_state, self.auto_driving])


    # Publish the current robot state
    def actuators_timer_callback(self):
        msg = String()

        # Python is silly and you have to declare global variables like this before using them
        global dig_button_toggled
        global digger_extend_button_toggled
        global offload_button_toggled
        global counter

        if self.current_state.value == states['Emergency_Stop']:
            msg.data = 'STOP_ALL_ACTUATORS'
        elif self.current_state.value == states['Auto_Dig']:
            msg.data = 'DIGGER_ON'
        elif self.current_state.value == states['Teleop']:
            if dig_button_toggled:
                msg.data += ' DIGGER_ON'
            elif not dig_button_toggled:
                msg.data += ' DIGGER_OFF'
            if offload_button_toggled:
                msg.data += ' OFFLOADING_ON'
            elif not offload_button_toggled:
                msg.data += ' OFFLOADING_OFF'
        elif self.current_state.value == states['Autonomous']:
            pass # TODO: Finish these Autonomous cases:
            # if condition_for_digging:
            #     msg.data += ' DIGGER_ON'
            # if not condition_for_digging:
            #     msg.data += ' DIGGER_OFF'
            # if condition_for_offloading:
            #     msg.data += ' OFFLOADING_ON'
            # if not condition_for_offloading:
            #     msg.data += ' OFFLOADING_OFF'
            

        self.actuators_publisher.publish(msg)
        
        if counter >= 20:
            self.get_logger().info('Publishing: "%s"' % msg.data) # Print to the terminal
            counter = 0
        counter += 1


    # When a joystick input is recieved, this callback updates the global power variables accordingly
    def joystick_callback(self, msg):
        
        # Python is silly and you have to declare global variables like this before using them
        global current_drive_power
        global current_turn_power
        global dig_button_toggled
        global camera_view_toggled
        global digger_extend_button_toggled
        global offload_button_toggled
        global camera0
        global camera1

        # Update our current driving powers
        current_drive_power = (msg.axes[RIGHT_JOYSTICK_VERTICAL_AXIS]) * max_drive_power # Forward power
        current_turn_power = (msg.axes[LEFT_JOYSTICK_HORIZONTAL_AXIS]) * max_turn_power # Turning power
        
        # Check if the digger button is pressed
        if msg.buttons[X_BUTTON] == 1 and buttons[X_BUTTON] == 0:
            dig_button_toggled = not dig_button_toggled
            
        # Check if the offloader button is pressed
        if msg.buttons[B_BUTTON] == 1 and buttons[B_BUTTON] == 0:
            offload_button_toggled = not offload_button_toggled
            
        # Check if the digger_extend button is pressed
        if msg.buttons[A_BUTTON] == 1 and buttons[A_BUTTON] == 0:
            digger_extend_button_toggled = not digger_extend_button_toggled
            if digger_extend_button_toggled:
                self.arduino.write(1) # Tell the Arduino to extend the linear actuator
            else:
                self.arduino.write(0) # Tell the Arduino to retract the linear actuator

        # Check if the autonomous digging button is pressed
        if msg.buttons[Y_BUTTON] == 1 and buttons[Y_BUTTON] == 0:
            if self.current_state.value == states["Teleop"]:
                self.current_state.value = states["Auto_Dig"]
                self.autonomous_digging_process.start() # Start the auto dig process
            elif self.current_state.value == states["Auto_Dig"]:
                self.current_state.value = states["Teleop"]
                self.autonomous_digging_process.terminate() # Terminate the auto dig process
                dig_button_toggled = False # When we enter teleop mode, start with the digger off
                offload_button_toggled = False # When we enter teleop mode, start with the offloader off
                
        # Check if the camera toggle button is pressed
        if msg.buttons[START_BUTTON] == 1 and buttons[START_BUTTON] == 0:
            camera_view_toggled = not camera_view_toggled
            if camera_view_toggled: # Start streaming /dev/video0 on port 5000
                if camera1 is not None:
                    os.killpg(os.getpgid(camera1.pid), signal.SIGTERM)
                    camera1 = None
                camera0 = subprocess.Popen('gst-launch-1.0 v4l2src device=/dev/video0 ! "video/x-raw,width=640,height=480,framerate=30/1" ! nvvidconv ! "video/x-raw(memory:NVMM),format=I420" ! omxh265enc bitrate=100000 ! "video/x-h265,stream-format=byte-stream" ! h265parse ! rtph265pay ! udpsink host=192.168.1.40 port=5000', shell=True, preexec_fn=os.setsid)
            else: # Start streaming /dev/video1 on port 5001
                if camera0 is not None:
                    os.killpg(os.getpgid(camera0.pid), signal.SIGTERM)
                    camera0 = None
                camera1 = subprocess.Popen('gst-launch-1.0 v4l2src device=/dev/video1 ! "video/x-raw,width=640,height=480,framerate=30/1" ! nvvidconv ! "video/x-raw(memory:NVMM),format=I420" ! omxh265enc bitrate=100000 ! "video/x-h265,stream-format=byte-stream" ! h265parse ! rtph265pay ! udpsink host=192.168.1.40 port=5000', shell=True, preexec_fn=os.setsid)

        # Update new button states
        for index in range(len(buttons)):
            buttons[index] = msg.buttons[index]


    # Decides what power (duty cycle) should be sent to the drive motors
    def drive_power_timer_callback(self):
        
        # Python is silly and you have to declare global variables like this before using them
        global current_drive_power
        global current_turn_power
        global counter
        
        drive_power_msg = Twist() # Create a new ROS2 msg

        # Default to 0 power for everything at first
        drive_power_msg.angular.x = 0.0  
        drive_power_msg.angular.y = 0.0
        drive_power_msg.angular.z = 0.0
        drive_power_msg.linear.x = 0.0
        drive_power_msg.linear.y = 0.0
        drive_power_msg.linear.z = 0.0

        if self.current_state.value == states['Teleop']:
            drive_power_msg.linear.x = current_drive_power # Forward power
            drive_power_msg.angular.z = current_turn_power # Turning power
        elif self.current_state.value == states['Auto_Dig'] and self.auto_driving.value:
            drive_power_msg.linear.x = dig_driving_power # Driving power while digging

        self.drive_power_publisher.publish(drive_power_msg)
        
        if counter >= 20:
            self.get_logger().info(f'Publishing Angular Power: {drive_power_msg.angular.z}, Linear Power: {drive_power_msg.linear.x}')


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
