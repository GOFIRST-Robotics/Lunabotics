import rclpy
from rclpy.node import Node

from nvblox_msgs.msg import DistanceMapSlice

# TODO: if numpy is too laggy, use cupy, the cuda accelerated version
import numpy as np

class combine_esdf(Node):

    def __init__(self):
        super().__init__('combine_esdf')
        self.publisher_ = self.create_publisher(DistanceMapSlice, '/combined_esdf', 10)

        self.above_ground_subscriber = self.create_subscription(
            DistanceMapSlice,
            '/nvblox_node/static_map_slice',
            self.above_ground_callback,
            10)
        
        self.below_ground_subscriber = self.create_subscription(
            DistanceMapSlice,
            '/nvblox_node/static_map_slice2',
            self.below_ground_callback,
            10)
        
        # Could eventually add more
        self.above_ground_costmap = None
        self.below_ground_costmap_one = None

        self.above_ground_subscriber
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0


    def above_ground_callback(self, msg):
        if (len(msg.data) == 0):
            return
        if (self.below_ground_costmap_one is None):
            self.publisher_.publish(msg)
        
        self.above_ground_costmap = np.array(msg.data)
        self.above_ground_costmap = self.above_ground_costmap.reshape(msg.height, msg.width)

        # Find valus on the cost map that are NOT 1000
        condition = self.below_ground_costmap_one == 1000
        # self.above_ground_costmap[condition]
        self.below_ground_costmap_one[condition] = 2

        inverse_costmap = np.full(self.below_ground_costmap_one.shape, 2) - self.below_ground_costmap_one

        # print(self.above_ground_costmap.shape)
        # print(np.average(inverse_costmap), np.average(self.above_ground_costmap))
        # print(np.amax(inverse_costmap), np.amin(inverse_costmap))
        # print(np.amax(self.above_ground_costmap), np.amin(self.above_ground_costmap))
        
        message = DistanceMapSlice()
        message = msg
        message.data = np.maximum(inverse_costmap, self.above_ground_costmap).flatten().tolist()
        message.height = msg.height
        message.width = msg.width
        self.publisher_.publish(message)

    def below_ground_callback(self, msg):
        if (len(msg.data) == 0):
            return
        self.below_ground_costmap_one = np.array(msg.data)
        self.below_ground_costmap_one = self.below_ground_costmap_one.reshape(msg.height, msg.width)

    def timer_callback(self):
        msg = DistanceMapSlice()
        msg.data = 'combine_esdf'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        if self.i == 10:
            self.destroy_node()
            rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    # print("hello world")
    node = combine_esdf()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()