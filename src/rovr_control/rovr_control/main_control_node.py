# Basic ROS Modules
import rclpy
from rclpy.node import Node

# ROS Formatted Message Types
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Vector3

# Define our robot's initial state
current_state = 'Emergency Stop'

# Define how fast we should drive forward while auto digging
dig_speed = 1.0

class PublishersAndSubscribers(Node):

    def __init__(self):
        super().__init__('publisher')

        # Actuators Publisher
        self.actuators_publisher = self.create_publisher(String, 'cmd_actuators', 10)
        actuators_timer_period = 0.5  # how often to publish measured in seconds
        self.actuators_timer = self.create_timer(actuators_timer_period, self.actuators_timer_callback)

        # Velocity Publisher
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        velocity_timer_period = 0.5  # how often to publish measured in seconds
        self.velocity_timer = self.create_timer(velocity_timer_period, self.velocity_timer_callback)

        # Robot Command Subscriber
        self.command_subscription = self.create_subscription(String, 'robot_command', self.command_callback, 10)

    def actuators_timer_callback(self):
        msg = String()
        if current_state == 'Auto Dig':
             msg.data = 'DIGGER_ON'
        elif current_state == 'Emergency Stop':
            msg.data = 'STOP_ALL_ACTUATORS'
        self.actuators_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def velocity_timer_callback(self):
        msg = Twist()

        # Default to 0 speed for everything
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        # Default to 0 speed for everything
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        if current_state == 'Teleop Drive':
            pass
            # TODO: Read desired speeds from joystick input
        elif current_state == 'Auto Drive':
            pass
            # TODO: Calculate driving speeds... somehow... lol
        elif current_state == 'Auto Dig':
            msg.linear.x = dig_speed 
        self.velocity_publisher.publish(msg)
        self.get_logger().info(f'Publishing Angular Speed: {msg.angular.x}, Linear Speed: {msg.linear.x}')


    def command_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        # Switch our current state if a new state has been requested
        if msg.data == 'Teleop Drive':
            current_state = 'Teleop Drive'
        elif msg.data == 'Auto Drive':
            current_state = 'Auto Drive'
        elif msg.data == 'Auto Dig':
            current_state = 'Auto Dig'
        elif msg.data == 'Emergency Stop':
            current_state = 'Emergency Stop'

        # Log the robot's current state
        self.get_logger().info('Current State: "%s"' % current_state)


def main(args=None):
    rclpy.init(args=args)

    print('Hello from the rovr_control package!')
    print('Initial State:', current_state)

    node = PublishersAndSubscribers()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
