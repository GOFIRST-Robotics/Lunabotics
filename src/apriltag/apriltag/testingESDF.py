import os
import rclpy
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nvblox_msgs.msg import DistanceMapSlice
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import matplotlib.pyplot as plt
import numpy as np
class ApriltagNode(Node):
    def __init__(self):
        super().__init__("apriltag_node")
        self.called = False
        self.create_subscription(DistanceMapSlice, "/nvblox_node/static_map_slice", self.displayTagCallBack, 10)
    # Publish transform if the tag is detected
    def displayTagCallBack(self, msg):
        if not self.called:
            esdf_slice = np.frombuffer(msg.data, dtype=np.float32)
            esdf_slice[esdf_slice==1000] = -1
            height = msg.height
            width = msg.width
            esdf_slice = esdf_slice.reshape((height, width))
            self.called = True
            plt.imshow(esdf_slice)
            plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    node = ApriltagNode()
    node.get_logger().info("Initializing the Apriltag node!")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()