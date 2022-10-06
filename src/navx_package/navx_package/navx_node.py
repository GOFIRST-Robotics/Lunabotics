import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class DataPublisher(Node):

    def __init__(self):
        super().__init__('data_publisher')
        self.publisher_ = self.create_publisher(String, 'navx_data', 10)

        timer_period = 0.5 # Delay in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1


def main(args=None):
    print('Hi from the navx_package! :)')
    rclpy.init(args=args)

    data_publisher = DataPublisher()

    rclpy.spin(data_publisher)

    data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
