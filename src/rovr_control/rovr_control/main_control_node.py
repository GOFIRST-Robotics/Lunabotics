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

# Define the joystick axes we want to use
buttons = [0] * 12
linear_axis = 3
angular_axis = 2
dig_button = 0; # TODO: Choose the right button
offload_button = 1; # TODO: Choose the right button
digger_extend_button = 2; # TODO: Choose the right button

dig_button_toggled = False
offload_button_toggled = False
digger_extend_button_toggled = False

# Define the possible states of our robot
states = {'Teleop': 0, 'Autonomous': 1, 'Auto_Dig': 2, 'Emergency_Stop': 3}
# Define our robot's initial state
current_state = states['Teleop']

# Define the maximum driving power of the robot (duty cycle)
dig_driving_power = 0.5 # The power to drive at when autonomously digging
max_drive_power = 1.0
max_turn_power = 1.0

# These global values are updated by joystick input
current_drive_power = 0.0
current_turn_power = 0.0

# Define our autonomous goal position # TODO: What are we even doing in terms of autonomous stuff lol
goal_x_absolute = 2  # Meters
goal_y_absolute = 2  # Meters
goal_orientation = 90  # Degrees

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
        
        # Goal Publisher
        self.goal_publisher = self.create_publisher(PoseStamped, 'Goal', 10)
        goal_timer_period = 0.05  # how often to publish measured in seconds
        self.goal_timer = self.create_timer(goal_timer_period, self.goal_timer_callback)

        # Robot Command Subscriber
        self.command_subscription = self.create_subscription(String, 'robot_command', self.command_callback, 10)
        
        # EKF Subscriber
        self.ekf_subscription = self.create_subscription(PoseWithCovarianceStamped, 'robot_pos_ekf/odom_combined', self.ekf_callback, 10)

        # Joystick Subscriber
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joystick_callback, 10)


    # Publish the current robot state
    def actuators_timer_callback(self):
        msg = String()

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
        self.get_logger().info('Publishing: "%s"' % msg.data)


    # When a joystick input is recieved, this callback updates the global power variables accordingly
    def joystick_callback(self, msg):
        global current_drive_power
        global current_turn_power

        # Update our current driving powers
        current_drive_power = (msg.axes[linear_axis]) * max_drive_power # Forward power
        current_turn_power = (msg.axes[angular_axis]) * max_turn_power # Turning power
        
        # Check if the digger button is pressed
        if msg.buttons[dig_button] == 1 and buttons[dig_button] == 0:
            dig_button_toggled = not dig_button_toggled
            
        # Check if the offloader button is pressed
        if msg.buttons[offload_button] == 1 and buttons[offload_button] == 0:
            offload_button_toggled = not offload_button_toggled
            
        # Check if the digger_extend button is pressed
        if msg.buttons[digger_extend_button] == 1 and buttons[digger_extend_button] == 0:
            digger_extend_button_toggled = not digger_extend_button_toggled
            
        # Update new button states
        for index in buttons:
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
        self.get_logger().info(f'Publishing Angular Power: {drive_power_msg.angular.z}, Linear Power: {drive_power_msg.linear.x}')


    # Publish our current goal to autonomously drive to # TODO: What are we even doing in terms of autonomous stuff lol
    def goal_timer_callback(self):

        # We only need to publish a goal if we are currently autonomous
        if current_state == states['Autonomous']:
            msg = PoseStamped()

            msg.pose.position.x = goal_x_absolute  # Meters
            msg.pose.position.y = goal_y_absolute  # Meters
            msg.pose.position.z = 0.0  # We can't move in this axis lol, the robot can't fly

            msg.pose.orientation.x = 0.0 # We can't move in this axis
            msg.pose.orientation.y = 0.0  # We can't move in this axis
            msg.pose.orientation.z = goal_orientation  # Degrees

            self.goal_publisher.publish(msg)
            self.get_logger().info(f'Publishing Position: {msg.pose.position}, Orientation: {msg.pose.orientation}')


    # Updates our current state when a command to change has been recieved
    # NOTE: The command message should be a String containing an integer (corresponding to the requested state)
    def command_callback(self, msg):
        self.get_logger().info('I received this message: "%i"' % msg.data)

        requested_state = msg

        # Switch our current state if a new (valid) state has been requested
        if 0 <= requested_state <= len(states) - 1:
            current_state = requested_state

        # Log the robot's current state
        self.get_logger().info('Current State is set to: "%i"' % current_state)


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
