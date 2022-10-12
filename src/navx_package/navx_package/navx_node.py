# Import rclpy so its Node class can be used
import rclpy
from rclpy.node import Node

# Imports the built-in string message type that the node uses to structure data
from std_msgs.msg import String


# Create a basic publisher class that is a child of Node
class DataPublisher(Node):

    def __init__(self):
        super().__init__('data_publisher')
        self.publisher_ = self.create_publisher(String, 'navx_data', 10)

        timer_period = 0.5  # Delay between messages in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0  # Keep track of how many messages have been sent

    def timer_callback(self):
        # Create the message to be sent
        msg = String()
        msg.data = f'NavX Data: {self.counter}'

        # Publish the message and increment counter by 1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1


# Print a 'Hello World' message and initialize the publisher
def main(args=None):
    print('Hi from the navx_package! :)')
    rclpy.init(args=args)

    data_publisher = DataPublisher()
    rclpy.spin(data_publisher)

    # Free up resources being used by the node
    data_publisher.destroy_node()
    rclpy.shutdown()


# Prevents unwanted side effects from occuring if this is imported as a module
if __name__ == '__main__':
    main()
