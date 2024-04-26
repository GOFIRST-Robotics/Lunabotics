import os
import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rovr_interfaces.srv import ResetOdom
from geometry_msgs.msg import TransformStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

import xml.etree.ElementTree as ET


class ApriltagNode(Node):
    def __init__(self):
        super().__init__("apriltag_node")
        current_dir = os.getcwd()

        self.declare_parameter("autonomous_field_type", "top")  # The type of field ("top", "bottom", "nasa")
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

        self.create_service(ResetOdom, "resetOdom", self.reset_callback)

        # Create a timer to broadcast the map -> odom transform
        self.timer = self.create_timer(0.1, self.broadcast_transform)

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
        if len(msg.detections) == 0:
            return

        tags = msg.detections
        for tag in tags:
            id = tag.id
            tree = ET.parse(self.file_path)
            root = tree.getroot()

            link = root[id]  # assumes tag 1 = home 1, tag 2 = home 2, 3 = berm 1 etc.

            xyz_elements = link.findall(".//origin[@xyz]")
            xyz_values = [element.attrib["xyz"] for element in xyz_elements]
            xyz = xyz_values[0].split(" ")

            # Lookup the odom to zed2i_camera_link tf from the tf buffer
            try:
                odom_to_tag_transform = self.tf_buffer.lookup_transform("odom", f"{tag.family}:{id}", rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(f"Could not transform odom to zed2i_camera_link: {ex}")
                return

            odom_to_tag_transform.child_frame_id = "odom"
            odom_to_tag_transform.header.frame_id = "map"
            odom_to_tag_transform.header.stamp = self.get_clock().now().to_msg()

            odom_to_tag_transform.transform.rotation.x = 0.0
            odom_to_tag_transform.transform.rotation.y = 0.0
            odom_to_tag_transform.transform.rotation.z = 1.0
            odom_to_tag_transform.transform.rotation.w = 0.0

            # We don't care about z because our robot can't move up or down
            odom_to_tag_transform.transform.translation.z = 0.0

            # Use the known map coordinates of the apriltag as an offset
            odom_to_tag_transform.transform.translation.x -= float(xyz[0])
            odom_to_tag_transform.transform.translation.y -= float(xyz[1])

            self.map_to_odom_tf = odom_to_tag_transform

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
