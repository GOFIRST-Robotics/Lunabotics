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
        # print(msg.origin, msg.height, msg.width)
        if (self.below_ground_costmap_one is None):
            self.publisher_.publish(msg)
            return
        self.above_ground_costmap = np.array(msg.data)
        self.above_ground_costmap = self.above_ground_costmap.reshape(msg.height, msg.width)

        # Set any null values in the costmap to an inaccessible area (before inverting)
        condition = self.below_ground_costmap_one > 2
        # self.above_ground_costmap[condition]
        self.below_ground_costmap_one[condition] = 2

        inverse_below_ground_costmap = np.full(self.below_ground_costmap_one.shape, 2) - self.below_ground_costmap_one

        # pad the below ground costmap, to give it the same dimensions as the above ground costmap
        # Set 
        width_diff = self.above_ground_costmap.shape[0] - inverse_below_ground_costmap.shape[0]
        height_diff = self.above_ground_costmap.shape[1] - inverse_below_ground_costmap.shape[1]
        # print(width_diff, height_diff)
        
        max_width = max(self.above_ground_costmap.shape[0], inverse_below_ground_costmap.shape[0])
        max_height = max(self.above_ground_costmap.shape[1], inverse_below_ground_costmap.shape[1])
        inverse_below_ground_costmap = np.pad(inverse_below_ground_costmap, ((max_width - inverse_below_ground_costmap.shape[0], 0), (max_height - inverse_below_ground_costmap.shape[1], 0)), 'constant', constant_values=(2))
        self.above_ground_costmap = np.pad(self.above_ground_costmap, ((max_width - self.above_ground_costmap.shape[0], 0), (max_height - self.above_ground_costmap.shape[1], 0)), 'constant', constant_values=(2))


        # if (width_diff > 0 and height_diff > 0):
        #     inverse_below_ground_costmap = np.pad(inverse_below_ground_costmap, ((height_diff, 0), (width_diff, 0)), 'constant', constant_values=(0))
        # elif (width_diff < 0 and height_diff < 0):
        #     self.above_ground_costmap = np.pad(self.above_ground_costmap, ((-height_diff, 0), (-width_diff, 0)), 'constant', constant_values=(0))

        # Create a message to publish the combined costmap
        message = DistanceMapSlice()
        message = msg # should be largely identical to the original message.
        if self.above_ground_costmap.shape == inverse_below_ground_costmap.shape:
            # print("it worked")
            inverse_below_ground_costmap = inverse_below_ground_costmap * 1000

            message.data = np.maximum(inverse_below_ground_costmap, self.above_ground_costmap).flatten().tolist()
            print("worked")
            # print(self.above_ground_costmap.shape, self.below_ground_costmap_one.shape)
        # Combine the costmaps, taking the larger 
        # message.data = np.maximum(inverse_costmap, self.above_ground_costmap).flatten().tolist()
        
        self.publisher_.publish(message)


    # Easily replicable for more below ground costmaps. Stack the costmaps into a 3D array
    # Probably useful to move the inversion logic to this function if this is the case.
    def below_ground_callback(self, msg):
        if (len(msg.data) == 0):
            return
        # print(msg.origin, msg.height, msg.width)
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