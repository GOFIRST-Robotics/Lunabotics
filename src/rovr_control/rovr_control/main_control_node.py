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

# Import our gamepad button mappings
from .gamepad_constants import *
# This is to help with button press detection
buttons = [0] * 12

dig_button_toggled = False
offload_button_toggled = False
digger_extend_button_toggled = False

# Define the possible states of our robot
states = {'Teleop': 0, 'Autonomous': 1, 'Auto_Dig': 2, 'Emergency_Stop': 3}
# Define our robot's initial state
current_state = states['Teleop']

# counter for not printing as fast
counter = 0

# Define the maximum driving power of the robot (duty cycle)
dig_driving_power = 0.5 # The power to drive at when autonomously digging
max_drive_power = 1.0
max_turn_power = 1.0

# These global values are updated by joystick input
current_drive_power = 0.0
current_turn_power = 0.0

class PublishersAndSubscribers(Node):

    def __init__(self):
        super().__init__('publisher')

        # Actuators Publisher
        self.actuators_publisher = self.create_publisher(String, 'cmd_actuators', 10)
        actuators_timer_period = 0.05  # how often to publish measured in seconds
        self.actuators_timer = self.create_timer(actuators_timer_period, self.actuators_timer_callback)
        
        # Drive Power Publisher
        self.drive_power_publisher = self.create_publisher(Twist, 'drive_power', 10)
        drive_power_timer_period = 0.05  # how often to publish measured in seconds
        self.drive_power_timer = self.create_timer(drive_power_timer_period, self.drive_power_timer_callback)
        
        # EKF Subscriber
        self.ekf_subscription = self.create_subscription(PoseWithCovarianceStamped, 'robot_pos_ekf/odom_combined', self.ekf_callback, 10)

        # Joystick Subscriber
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joystick_callback, 10)


    # Publish the current robot state
    def actuators_timer_callback(self):
        msg = String()

        global counter

        global dig_button_toggled
        global digger_extend_button_toggled
        global offload_button_toggled

        if current_state == states['Emergency_Stop']:
            msg.data = 'STOP_ALL_ACTUATORS'
        elif current_state == states['Auto_Dig']:
            msg.data = 'DIGGER_ON BEGIN_DIG_PROCEDURE' # TODO: Implement BEGIN_DIG_PROCEDURE in motor_control node
        elif current_state == states['Teleop']:
            if dig_button_toggled:
                msg.data += ' DIGGER_ON'
            elif not dig_button_toggled:
                msg.data += ' DIGGER_OFF'
            if offload_button_toggled:
                msg.data += ' OFFLOADING_ON'
            elif not offload_button_toggled:
                msg.data += ' OFFLOADING_OFF'
            if digger_extend_button_toggled:
                msg.data += ' EXTEND_DIGGER'
            elif not digger_extend_button_toggled:
                msg.data += ' RETRACT_DIGGER'
        elif current_state == states['Autonomous']:
            pass # TODO: Finish these Autonomous cases:
            # if condition for digging
                # msg.data += ' DIGGER_ON'
            # if NOT condition for diggging
                # msg.data += ' DIGGER_OFF'
            # if condition for offloading
                # msg.data += ' OFFLOADING_ON'
            # if NOT condition for offloading
                # msg.data += ' OFFLOADING_OFF'
            # if condition for extending digger
                # msg.data += ' EXTEND_DIGGER'
            # if condition for retracting digger
                # msg.data += ' RETRACT_DIGGER'

        self.actuators_publisher.publish(msg)

        if counter >= 5:
            self.get_logger().info('Publishing: "%s"' % msg.data)
            counter = 0
        counter += 1


    # When a joystick input is recieved, this callback updates the global power variables accordingly
    def joystick_callback(self, msg):
        global current_drive_power
        global current_turn_power

        global dig_button_toggled
        global digger_extend_button_toggled
        global offload_button_toggled

        global current_state

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

        # Check if the autonomous digging button is pressed
        if msg.buttons[Y_BUTTON] == 1 and buttons[Y_BUTTON] == 0:
            if current_state == states["Teleop"]:
                current_state = states["Auto_Dig"]
            elif current_state == states["Auto_Dig"]:
                current_state = states["Teleop"]

            
        # Update new button states
        for index in range(len(buttons)):
            buttons[index] = msg.buttons[index]

    # Decides what power (duty cycle) should be sent to the drive motors
    def drive_power_timer_callback(self):
        global current_drive_power
        global current_turn_power
        
        drive_power_msg = Twist() # Create a new ROS2 msg

        # Default to 0 power for everything at first
        drive_power_msg.angular.x = 0.0  
        drive_power_msg.angular.y = 0.0
        drive_power_msg.angular.z = 0.0
        drive_power_msg.linear.x = 0.0
        drive_power_msg.linear.y = 0.0
        drive_power_msg.linear.z = 0.0

        if current_state == states['Teleop']:
            drive_power_msg.linear.x = current_drive_power # Forward power
            drive_power_msg.angular.z = current_turn_power # Turning power
        elif current_state == states['Auto_Dig']:
            drive_power_msg.linear.x = dig_driving_power # Driving power while digging

        self.drive_power_publisher.publish(drive_power_msg)
        #self.get_logger().info(f'Publishing Angular Power: {drive_power_msg.angular.z}, Linear Power: {drive_power_msg.linear.x}')


    # EKF stuff # TODO: Finish this callback. How do we implement EKF?
    def ekf_callback(self, msg):
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z

        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z

        covariance = msg.pose.covariance

        self.get_logger().info(
            f'Received a message with covariance of: {covariance}')


def main(args=None):
    rclpy.init(args=args)

    print('Hello from the rovr_control package!')
    print('Initial Robot State:', current_state)

    node = PublishersAndSubscribers()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == '__main__':
    main()
