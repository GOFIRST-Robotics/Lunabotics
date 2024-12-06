import os
import rclpy
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

import xml.etree.ElementTree as ET
import numpy as np


class ApriltagNode(Node):
    def __init__(self):
        super().__init__("apriltag_node")
        current_dir = os.getcwd()

        self.declare_parameter("autonomous_field_type", "nasa")  # The type of field ("top", "bottom", "nasa")
        field_type = self.get_parameter("autonomous_field_type").value
        paths = {
            "top": "src/apriltag/apriltag/apriltag_location_ucf_top.urdf.xarco",
            "bottom": "src/apriltag/apriltag/apriltag_location_ucf_bot.urdf.xarco",
            "nasa": "src/apriltag/apriltag/apriltag_location_nasa.urdf.xarco",
        }
        relative_path = paths[field_type]
        self.file_path = os.path.join(current_dir, relative_path)

        self.map_to_odom_tf = None

        self.map_transform = TransformStamped()
        self.map_transform.child_frame_id = "odom"
        self.map_transform.header.frame_id = "map"
        self.map_transform.transform.translation.x = 0.0
        self.map_transform.transform.translation.y = 0.0
        self.map_transform.transform.translation.z = 0.0
        self.map_transform.transform.rotation.x = 0.0
        self.map_transform.transform.rotation.y = 0.0
        self.map_transform.transform.rotation.z = 0.0
        self.map_transform.transform.rotation.w = 1.0

        self.transforms = self.create_subscription(AprilTagDetectionArray, "/tag_detections", self.tagDetectionSub, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, "resetOdom", self.reset_callback)

        # Create a timer to broadcast the map -> odom transform
        self.timer = self.create_timer(0.01, self.broadcast_transform)

    # Service callback definition
    def reset_callback(self, request, response):
        """Run once, return success/ fail"""
        response.success = bool(self.postTransform(self.map_to_odom_tf))
        return response

    # Publish transform if the tag is detected
    def postTransform(self, tag):
        if tag and (self.get_clock().now().to_msg().sec == tag.header.stamp.sec):
            self.get_logger().info(str("Resetting the map -> odom TF"))
            self.map_transform = tag
            return True
        return False
    
    def average_quaternions(self, quaternions):
        # Stack quaternions into a matrix
        q_matrix = np.array(quaternions)
        
        # Compute the mean of the quaternions as if they are points in 4D space
        q_mean = np.mean(q_matrix, axis=0)
        
        # Normalize the resulting quaternion
        q_mean /= np.linalg.norm(q_mean)
        
        return q_mean


    def tagDetectionSub(self, msg):
        if len(msg.detections) == 0:
            return

        # Initialize cumulative sums and counter
        cumulative_position = np.zeros(3)  # For x, y, z
        cumulative_quaternions = []  # List to store quaternions
        tag_count = 0

        for tag in msg.detections:
            id = tag.id
            try:
                tree = ET.parse(self.file_path)
                root = tree.getroot()
                link = root[id - 1]  # assumes tag 1 = home 1, tag 2 = home 2, etc.
            except (ET.ParseError, IndexError) as e:
                self.get_logger().warn(f"Error parsing XML or accessing tag data: {e}")
                continue

            # Extract known map coordinates of the tag
            xyz_elements = link.findall(".//origin[@xyz]")
            xyz_values = [element.attrib["xyz"] for element in xyz_elements]
            xyz = [float(coord) for coord in xyz_values[0].split(" ")]

            rpy_elements = link.findall(".//origin[@rpy]")
            rpy_values = [element.attrib["rpy"] for element in rpy_elements]
            rpy = [float(angle) for angle in rpy_values[0].split(" ")]

            # Lookup the odom to detected tag transform
            try:
                odom_to_tag_transform = self.tf_buffer.lookup_transform("odom", f"{tag.family}:{id}", rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(f"Could not transform odom to detected tag: {ex}")
                continue

            # Adjust position
            position = np.array([
                odom_to_tag_transform.transform.translation.x - xyz[0],
                odom_to_tag_transform.transform.translation.y - xyz[1],
                0.0,  # Assuming a 2D map
            ])
            cumulative_position += position

            # Adjust rotation
            rotation_quaternion = R.from_euler("xyz", rpy, degrees=True).as_quat()
            current_quaternion = np.array([
                odom_to_tag_transform.transform.rotation.x,
                odom_to_tag_transform.transform.rotation.y,
                odom_to_tag_transform.transform.rotation.z,
                odom_to_tag_transform.transform.rotation.w,
            ])
            rotated_quaternion = (R.from_quat(current_quaternion) * R.from_quat(rotation_quaternion)).as_quat()
            cumulative_quaternions.append(rotated_quaternion)

            tag_count += 1

        if tag_count > 0:
            # Compute the average position
            avg_position = cumulative_position / tag_count

            # Compute the average rotation using the helper function
            avg_rotation = self.average_quaternions(cumulative_quaternions)

            # Update the map_to_odom_tf with the averages
            self.map_to_odom_tf.transform.translation.x = avg_position[0]
            self.map_to_odom_tf.transform.translation.y = avg_position[1]
            self.map_to_odom_tf.transform.translation.z = avg_position[2]
            self.map_to_odom_tf.transform.rotation.x = avg_rotation[0]
            self.map_to_odom_tf.transform.rotation.y = avg_rotation[1]
            self.map_to_odom_tf.transform.rotation.z = avg_rotation[2]
            self.map_to_odom_tf.transform.rotation.w = avg_rotation[3]


    def broadcast_transform(self):
        """Broadcasts the map -> odom transform"""
        self.map_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_transform)


def main(args=None):
    rclpy.init(args=args)

    node = ApriltagNode()
    node.get_logger().info("Initializing the Apriltag node!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
