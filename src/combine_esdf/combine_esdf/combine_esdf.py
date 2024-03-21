import rclpy
from rclpy.node import Node

from nvblox_msgs.msg import DistanceMapSlice
from sensor_msgs.msg import PointCloud2, PointField

# TODO: if numpy is too laggy, use cupy, the cuda accelerated version
# Doesn't seem like its a problem so far though.
import numpy as np

class combine_esdf(Node):

    def __init__(self):
        super().__init__('combine_esdf')
        self.costmap_publisher = self.create_publisher(DistanceMapSlice, '/combined_esdf', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/combined_esdf_pointcloud', 10)

        # self.above_ground_subscriber = self.create_subscription(
        #     DistanceMapSlice,
        #     '/nvblox_node/static_map_slice',
        #     self.above_ground_callback,
        #     10)
        
        # self.below_ground_subscriber = self.create_subscription(
        #     DistanceMapSlice,
        #     '/nvblox_node/static_map_slice2',
        #     self.below_ground_callback,
        #     10)
        
        self.above_ground_pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/nvblox_node/static_esdf_pointcloud',
            self.above_ground_pointcloud_callback,
            10)
        
        self.below_ground_pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/nvblox_node/static_esdf_pointcloud2',
            self.below_ground_pointcloud_callback,
            10)

        # Could eventually add more below ground costmaps here
        self.above_ground_costmap = None
        self.below_ground_costmap_one = None

        self.above_ground_pointcloud = None
        self.below_ground_pointcloud = None


    def above_ground_callback(self, msg):
        if (len(msg.data) == 0):
            return
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
        max_width = max(self.above_ground_costmap.shape[0], inverse_below_ground_costmap.shape[0])
        max_height = max(self.above_ground_costmap.shape[1], inverse_below_ground_costmap.shape[1])
        inverse_below_ground_costmap = np.pad(inverse_below_ground_costmap, ((max_width - inverse_below_ground_costmap.shape[0], 0), (max_height - inverse_below_ground_costmap.shape[1], 0)), 'constant', constant_values=(2))
        self.above_ground_costmap = np.pad(self.above_ground_costmap, ((max_width - self.above_ground_costmap.shape[0], 0), (max_height - self.above_ground_costmap.shape[1], 0)), 'constant', constant_values=(2))

        # Create a message to publish the combined costmap
        message = DistanceMapSlice()
        message = msg # should be largely identical to the original message.
        if self.above_ground_costmap.shape == inverse_below_ground_costmap.shape:
            inverse_below_ground_costmap = inverse_below_ground_costmap * 1000
            message.data = np.maximum(inverse_below_ground_costmap, self.above_ground_costmap).flatten().tolist()
            self.costmap_publisher.publish(message)


    # Easily replicable for more below ground costmaps. Stack the costmaps into a 3D array
    # Probably useful to move the inversion logic to this function if this is the case.
    def below_ground_callback(self, msg):
        if (len(msg.data) == 0):
            return
        
        self.below_ground_costmap_one = np.array(msg.data)
        self.below_ground_costmap_one = self.below_ground_costmap_one.reshape(msg.height, msg.width)




    def above_ground_pointcloud_callback(self, msg):
        if (len(msg.data) == 0):
            return
        uint8_cloud = np.frombuffer(msg.data, dtype=np.uint8)
        float_values = uint8_cloud.view(dtype=np.float32)
        slice_indices = np.arange(3, len(float_values), 4)
        float_values[slice_indices] = 2 - float_values[slice_indices]
        message = msg
        message.data = np.frombuffer(float_values.tobytes(), dtype=np.uint8).tolist()
        self.pointcloud_publisher.publish(message)

    def below_ground_pointcloud_callback(self, msg):
        if (len(msg.data) == 0):
            return
        self.below_ground_pointcloud = np.array(msg.data)



def main(args=None):
    rclpy.init(args=args)
    # print("hello world")
    node = combine_esdf()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()