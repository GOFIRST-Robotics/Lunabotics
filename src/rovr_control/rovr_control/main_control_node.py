# Basic ROS Modules
import rclpy
from rclpy.node import Node

# ROS Formatted Message Types
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

# Define the possible states that our robot can be in
states = {'Teleop Drive': 0, 'Auto Drive': 1, 'Auto Dig': 2, 'Emergency Stop': 3}
# Define our robot's initial state
current_state = states['Teleop Drive']

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
        # Goal Publisher
        self.goal_publisher = self.create_publisher(PoseStamped, 'Goal', 10)
        goal_timer_period = 0.5  # how often to publish measured in seconds
        self.goal_timer = self.create_timer(goal_timer_period, self.goal_timer_callback)

        # Robot Command Subscriber
        self.command_subscription = self.create_subscription(String, 'robot_command', self.command_callback, 10)
        # EKF Subscriber
        self.ekf_subscription = self.create_subscription(PoseWithCovarianceStamped, 'robot_pos_ekf/odom_combined', self.ekf_callback, 10)
        # Velocity Subscriber
        self.velocity_subscription = self.create_subscription(Twist, 'cmd_vel', self.velocity_callback, 10)

    def actuators_timer_callback(self):
        msg = String()

        if current_state == states['Auto Dig']:
            msg.data = 'DIGGER_ON'
            self.actuators_publisher.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        elif current_state == states['Emergency Stop']:
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

        if current_state == states['Teleop Drive']:
            pass
            # TODO: Read desired speeds from joystick input
        elif current_state == states['Auto Drive']:
            pass
            # TODO: Calculate driving speeds... somehow... lol
        elif current_state == states['Auto Dig']:
            msg.linear.x = dig_speed 
        self.velocity_publisher.publish(msg)
        self.get_logger().info(f'Publishing Angular Speed: {msg.angular.x}, Linear Speed: {msg.linear.x}')


    def goal_timer_callback(self):

        # We only need to publish a goal if we are currently autonomous
        if current_state == states['Auto Drive']:
            msg = PoseStamped()

            msg.pose.position.x = 0.0 # TODO: How do we calculate this???
            msg.pose.position.y = 0.0 # TODO: How do we calculate this???
            msg.pose.position.z = 0.0 # TODO: How do we calculate this???

            msg.pose.orientation.x = 0.0 # TODO: How do we calculate this???
            msg.pose.orientation.y = 0.0 # TODO: How do we calculate this???
            msg.pose.orientation.z = 0.0 # TODO: How do we calculate this???
        
            self.goal_publisher.publish(msg)
            self.get_logger().info(f'Publishing Position: {msg.pose.position}, Orientation: {msg.pose.orientation}')


    def command_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        # Switch our current state if a new (valid) state has been requested
        if 0 <= msg.data <= len(states) - 1:
            current_state = msg.data

        # Log the robot's current state
        self.get_logger().info('Current State: "%s"' % current_state)

    
    def ekf_callback(self, msg):
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z

        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z

        covariance = msg.pose.covariance

        self.get_logger().info(f'Received a message with covariance of: {covariance}')


    def velocity_callback(self, msg):
        angular_velocity = msg.angular
        linear_velocity = msg.linear

        self.get_logger().info(f'Received a velocity message with Angular: {angular_velocity} and Linear: {linear_velocity}')
        
        
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
