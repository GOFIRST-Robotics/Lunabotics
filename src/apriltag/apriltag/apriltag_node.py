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

        # Get the root of the XML tree for the current field type
        try:
            tree = ET.parse(self.file_path)
            self.xml_root = tree.getroot()
        except ET.ParseError as e:
            self.get_logger().warn(f"Error parsing XML: {e}")

        # Iterate through all apriltags in the XML tree and store their known map coordinates
        self.apriltag_map_coords = {}
        for link in self.xml_root.findall(".//link"):
            tag_id = int(link.attrib["name"].split("_")[-1])

            # Extract known map coordinates of the tag
            xyz_elements = link.findall(".//origin[@xyz]")
            xyz_values = [element.attrib["xyz"] for element in xyz_elements]
            xyz = [float(coord) for coord in xyz_values[0].split(" ")]

            # Extract known map orientation of the tag
            rpy_elements = link.findall(".//origin[@rpy]")
            rpy_values = [element.attrib["rpy"] for element in rpy_elements]
            rpy = [float(angle) for angle in rpy_values[0].split(" ")]

            self.apriltag_map_coords[tag_id] = (xyz, rpy)

        self.map_to_odom_tf = TransformStamped()
        self.map_to_odom_tf.child_frame_id = "odom"
        self.map_to_odom_tf.header.frame_id = "map"

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

    def tagDetectionSub(self, msg):
        if len(msg.detections) > 0:
            tags = msg.detections
            # Initialize cumulative sums and counter
            cumulative_position = np.zeros(3, None)
            tag_count = 0
            for tag in tags:
                # Extract the id of the detected tag
                id = tag.id
                # Extract the known map coordinates of the detected tag
                xyz, rpy = self.apriltag_map_coords[id]

                # Lookup the odom to detected tag tf from the tf buffer
                try:
                    odom_to_tag_transform = self.tf_buffer.lookup_transform(
                        "odom", f"{tag.family}:{id}", rclpy.time.Time()
                    )
                except TransformException as ex:
                    self.get_logger().warn(f"Could not transform odom to the detected tag: {ex}")
                    continue

                # Use the known map coordinates of the apriltag as an offset
                position = np.array(
                    [
                        odom_to_tag_transform.transform.translation.x - xyz[0],
                        odom_to_tag_transform.transform.translation.y - xyz[1],
                        0.0,  # Assuming a 2D map
                    ]
                )

                # Apply a known rotation to the transform
                rotation_quaternion = R.from_euler(
                    "xyz", [float(rpy[0]), float(rpy[1]), float(rpy[2])], degrees=True
                ).as_quat()
                current_quaternion = R.from_quat(
                    [
                        odom_to_tag_transform.transform.rotation.x,
                        odom_to_tag_transform.transform.rotation.y,
                        odom_to_tag_transform.transform.rotation.z,
                        odom_to_tag_transform.transform.rotation.w,
                    ]
                ).as_quat()
                rotated_quaternion = current_quaternion * rotation_quaternion  # Multiply the quaternions

                # Apply the rotation to the position before adding it to the cumulative sum
                # Then, we can just average the positions and use an identity quaternion for the rotation
                cumulative_position += R.from_quat(rotated_quaternion).apply(position)
                tag_count += 1

            if tag_count > 0:
                # Compute the average position
                avg_position = cumulative_position / tag_count

                # Update the map_to_odom_tf with the averages
                self.map_to_odom_tf.transform.translation.x = avg_position[0]
                self.map_to_odom_tf.transform.translation.y = avg_position[1]
                self.map_to_odom_tf.transform.translation.z = avg_position[2]
                self.map_to_odom_tf.transform.rotation.x = 0.0
                self.map_to_odom_tf.transform.rotation.y = 0.0
                self.map_to_odom_tf.transform.rotation.z = 0.0
                self.map_to_odom_tf.transform.rotation.w = 1.0

                self.map_to_odom_tf.header.stamp = self.get_clock().now().to_msg()

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
