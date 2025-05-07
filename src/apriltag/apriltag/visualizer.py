import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from threading import Lock
import time

from scipy import ndimage

# Import message types
from nvblox_msgs.msg import DistanceMapSlice

class DistanceMapVisualizer(Node):
    def __init__(self):
        super().__init__('distance_map_visualizer')
        self.subscription = self.create_subscription(
            DistanceMapSlice,
            '/nvblox_node/static_map_slice',
            self.listener_callback,
            10)
        

        self.lock = Lock()
        self.distance_map = None

    def listener_callback(self, msg):
        with self.lock:
            # Extract the distance map from the message
            self.distance_map = np.array(msg.data).reshape((msg.height, msg.width))
            self.distance_map = np.clip(self.distance_map, 0, 10)
            self.distance_map[self.distance_map == 10] = -1

            # self.distance_map = np.where(self.distance_map == 0, np.nan, self.distance_map)

            if self.distance_map is None:
                return

            self.gradient_map = np.abs(ndimage.sobel(self.distance_map, axis=0)) + np.abs(ndimage.sobel(self.distance_map, axis=1))

            # self.gradient_map[self.gradient_map > 0.1] = 1


            # self.averaged_gradient_map = self.gradient_map.copy()
            self.distance_map = np.pad(self.distance_map, pad_width=1, mode='constant', constant_values=0)
            self.gradient_map = np.pad(self.gradient_map, pad_width=1, mode='constant', constant_values=0)
            self.averaged_gradient_map = self.gradient_map.copy()
            for i in range(self.gradient_map.shape[0]):
                for j in range(self.gradient_map.shape[1]):
                    if self.distance_map[i, j] == -1:
                        self.averaged_gradient_map[i, j] = 0
                    else:
                        neighborhood = self.distance_map[i-1:i+2, j-1:j+2]
                        if neighborhood.size > 0 and -1 in neighborhood:
                            self.averaged_gradient_map[i, j] = 0
        

            print("hiasd")

            
            self.averaged_gradient_map[self.averaged_gradient_map > 0.24] = 1


            plt.imshow(self.averaged_gradient_map, cmap='hot', interpolation='nearest')
            plt.imshow(self.averaged_gradient_map, interpolation='nearest')
            plt.title('Distance Map Slice')
            plt.pause(0.01)
            plt.show()

def main(args=None):
    rclpy.init(args=args)

    node = DistanceMapVisualizer()
    node.get_logger().info("Initializing the Apriltag node!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()