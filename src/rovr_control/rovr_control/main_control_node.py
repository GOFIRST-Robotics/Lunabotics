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

# Define the joystick axes we want to use
linear_axis = 1
angular_axis = 3

# Define the possible states of our robot
states = {'Teleop': 0, 'Autonomous': 1, 'Auto_Dig': 2, 'Emergency_Stop': 3}
# Define our robot's initial state
current_state = states['Auto_Dig']

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
        self.actuators_publisher = self.create_publisher(String, 'cmd_actuators', 1)
        actuators_timer_period = 0.5  # how often to publish measured in seconds
        self.actuators_timer = self.create_timer(actuators_timer_period, self.actuators_timer_callback)
        
        # Drive Power Publisher
        self.drive_power_publisher = self.create_publisher(Twist, 'drive_power', 1)
        drive_power_timer_period = 0.5  # how often to publish measured in seconds
        self.drive_power_timer = self.create_timer(drive_power_timer_period, self.drive_power_timer_callback)
        
        # Goal Publisher
        self.goal_publisher = self.create_publisher(PoseStamped, 'Goal', 1)
        goal_timer_period = 0.5  # how often to publish measured in seconds
        self.goal_timer = self.create_timer(goal_timer_period, self.goal_timer_callback)

        # Robot Command Subscriber
        self.command_subscription = self.create_subscription(String, 'robot_command', self.command_callback, 1)
        
        # EKF Subscriber
        self.ekf_subscription = self.create_subscription(PoseWithCovarianceStamped, 'robot_pos_ekf/odom_combined', self.ekf_callback, 1)

        # Joystick Subscriber
        self.joy_subscription = self.create_subscription(String, 'joy', self.joystick_callback, 1)


    # Publish the current robot state
    def actuators_timer_callback(self):
        msg = String()

        if current_state == states['Auto_Dig']:
            msg.data = 'DIGGER_ON'
        elif current_state == states['Emergency Stop']:
            msg.data = 'STOP_ALL_ACTUATORS'

        self.actuators_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


    # When a joystick input is recieved, this callback updates the global power variables accordingly
    def joystick_callback(self, msg):
        global current_drive_power
        global current_turn_power

        current_drive_power = (msg.axes[linear_axis]) * max_drive_power # Forward power
        current_turn_power = (msg.axes[angular_axis]) * max_turn_power # Turning power
        

    # Decides what power (duty cycle) should be sent to the drive motors
    def drive_power_timer_callback(self):
        global current_drive_power
        global current_turn_power
        drive_power_msg = Twist()

        # Default to 0 power for everything
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


    # Publish our currently goal to autonomously drive to
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


    # EKF stuff # TODO: Finish this callback
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
