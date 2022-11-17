# Basic ROS Modules
import rclpy
from rclpy.node import Node

# ROS Formatted Message Types
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

# Define the possible states that our robot can be in
states = {'Teleop Drive': 0, 'Auto Drive': 1,
          'Auto Dig': 2, 'Emergency Stop': 3}
# Define our robot's initial state
current_state = states['Teleop Drive']

# Define the maximum driving speeds of the robot
dig_driving_speed = 0.5
robot_drive_speed = 0.5
robot_turn_speed = 0.5

# Define our autonomous goal position
goal_x_absolute = 2  # Meters
goal_y_absolute = 2  # Meters
goal_orientation = 90  # Degrees

# Initialize variables to keep track of joystick input
joystick_input_linear = 0.0
joystick_input_angular = 0.0


class PublishersAndSubscribers(Node):

    def __init__(self):
        super().__init__('publisher')

        # Actuators Publisher
        self.actuators_publisher = self.create_publisher(
            String, 'cmd_actuators', 10)
        actuators_timer_period = 0.5  # how often to publish measured in seconds
        self.actuators_timer = self.create_timer(
            actuators_timer_period, self.actuators_timer_callback)
        
        # Velocity Publisher
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        velocity_timer_period = 0.5  # how often to publish measured in seconds
        self.velocity_timer = self.create_timer(
            velocity_timer_period, self.velocity_timer_callback)
        
        # Goal Publisher
        self.goal_publisher = self.create_publisher(PoseStamped, 'Goal', 10)
        goal_timer_period = 0.5  # how often to publish measured in seconds
        self.goal_timer = self.create_timer(
            goal_timer_period, self.goal_timer_callback)

        # Robot Command Subscriber
        self.command_subscription = self.create_subscription(
            String, 'robot_command', self.command_callback, 10)
        
        # EKF Subscriber
        self.ekf_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'robot_pos_ekf/odom_combined', self.ekf_callback, 10)
        
        # Velocity Subscriber
        self.velocity_subscription = self.create_subscription(
            Twist, 'cmd_vel', self.velocity_callback, 10)

    def actuators_timer_callback(self):
        msg = String()

        if current_state == states['Auto Dig']:
            msg.data = 'DIGGER_ON'
        elif current_state == states['Emergency Stop']:
            msg.data = 'STOP_ALL_ACTUATORS'
        elif current_state == states['Auto Drive']:
            msg.data = 'DIGGER_OFF'
        elif current_state == states['Teleop Drive']:
            msg.data = 'DIGGER_OFF'
        self.actuators_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def velocity_timer_callback(self):
        msg = Twist()

        global joystick_input_linear
        global joystick_input_angular

        # Default to 0 speed for everything
        msg.angular.x = 0.0  # I've defined this to be the axis we use
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        # Default to 0 speed for everything
        msg.linear.x = 0.0  # I've defined this to be forward/backward
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        if current_state == states['Teleop Drive']:
            msg.linear.x = joystick_input_linear * robot_drive_speed # I've defined this to be the axis we use
            msg.angular.x = joystick_input_angular * robot_turn_speed # I've defined this to be forward
        elif current_state == states['Auto Dig']:
            msg.linear.x = dig_driving_speed
        self.velocity_publisher.publish(msg)
        self.get_logger().info(
            f'Publishing Angular Speed: {msg.angular.x}, Linear Speed: {msg.linear.x}')

    def goal_timer_callback(self):

        # We only need to publish a goal if we are currently autonomous
        if current_state == states['Auto Drive']:
            msg = PoseStamped()

            msg.pose.position.x = goal_x_absolute  # Meters
            msg.pose.position.y = goal_y_absolute  # Meters
            msg.pose.position.z = 0.0  # We can't move in this axis lol, the robot can't fly

            msg.pose.orientation.x = goal_orientation  # Degrees
            msg.pose.orientation.y = 0.0  # We can't move in this axis
            msg.pose.orientation.z = 0.0  # We can't move in this axis

            self.goal_publisher.publish(msg)
            self.get_logger().info(
                f'Publishing Position: {msg.pose.position}, Orientation: {msg.pose.orientation}')

    # NOTE: The command message should be a String formatted like "<(int)state>, <(double)jopystick_input_linear>, <(double)joystick_input_angular>"

    def command_callback(self, msg):
        self.get_logger().info('I received this message: "%i"' % msg.data)

        global joystick_input_linear
        global joystick_input_angular

        # Split the message into the useful parts
        message_parts = msg.data.split(", ")
        requested_state = message_parts[0]
        joystick_input_linear = message_parts[1]
        joystick_input_angular = message_parts[2]

        # Switch our current state if a new (valid) state has been requested
        if 0 <= requested_state <= len(states) - 1:
            current_state = requested_state

        # Log the robot's current state
        self.get_logger().info('Current State is set to: "%i"' % current_state)

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

    def velocity_callback(self, msg):
        angular_velocity = msg.angular
        linear_velocity = msg.linear

        self.get_logger().info(
            f'Received a velocity message with Angular: {angular_velocity} and Linear: {linear_velocity}')


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
