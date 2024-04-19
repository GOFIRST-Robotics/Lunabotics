import os
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from rovr_interfaces.srv import ResetOdom
from geometry_msgs.msg import TransformStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

import xml.etree.ElementTree as ET

"""Make sure to turn on the camera using 
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py
or nothing here will work"""


class ApriltagNode(Node):
    def __init__(self):
        super().__init__("apriltag_node")
        current_dir = os.getcwd()

        self.declare_parameter('autonomous_field_type', 'top') # The type of field ("top", "bottom", "nasa")
        field_type = self.get_parameter('autonomous_field_type').value
        paths = {
            "top": "src/apriltag/apriltag/apriltag_location_ucf_top.urdf.xarco",
            "bottom": "src/apriltag/apriltag/apriltag_location_ucf_bot.urdf.xarco",
            "nasa": "src/apriltag/apriltag/apriltag_location_nasa.urdf.xarco"
        }
        relative_path = paths[field_type]
        self.file_path = os.path.join(current_dir, relative_path)


        self.averagedTag = None

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

        self.create_service(ResetOdom, "resetOdom", self.reset_callback)

        # Create a timer to broadcast the map -> odom transform
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    # Service callback definition
    def reset_callback(self, request, response):
        """Run once, return success/ fail"""
        response.success = bool(self.postTransform(self.averagedTag))
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
        transforms = []
        for tag in tags:
            t = TransformStamped()
            t.child_frame_id = "odom"
            t.header.frame_id = "map"
            t.header.stamp = self.get_clock().now().to_msg()

            id = tag.id
            tree = ET.parse(self.file_path)
            root = tree.getroot()

            link = root[id]  # assumes tag 1 = home 1, tag 2 = home 2, 3 = berm 1 etc.

            xyz_elements = link.findall(".//origin[@xyz]")
            xyz_values = [element.attrib["xyz"] for element in xyz_elements]
            xyz = xyz_values[0].split(" ")

            rpy_elements = link.findall(".//origin[@rpy]")
            rpy_values = [element.attrib["rpy"] for element in rpy_elements]
            rpy = rpy_values[0].split(" ")

            t.transform.translation.x = tag.pose.pose.pose.position.x - float(xyz[0])
            t.transform.translation.y = tag.pose.pose.pose.position.y - float(xyz[1])
            t.transform.translation.z = tag.pose.pose.pose.position.z - float(xyz[2])
            t.transform.rotation.x = tag.pose.pose.pose.orientation.x - float(rpy[0])
            t.transform.rotation.y = tag.pose.pose.pose.orientation.y - float(rpy[1])
            t.transform.rotation.z = tag.pose.pose.pose.orientation.z - float(rpy[2])

            transforms.append(t)

        self.averagedTag = TransformStamped()
        self.averagedTag.child_frame_id = "odom"
        self.averagedTag.header.frame_id = "map"
        self.averagedTag.header.stamp = self.get_clock().now().to_msg()
        self.averagedTag = self.averageTransforms(transforms, self.averagedTag)

    # TODO: Consider using an EKF instead of just averaging
    def averageTransforms(self, transforms, t):
        # Computes the average of a list of transforms
        x = 0
        y = 0
        z = 0
        qx = 0
        qy = 0
        qz = 0
        for transform in transforms:
            x += transform.transform.translation.x
            y += transform.transform.translation.y
            z += transform.transform.translation.z
            qx += transform.transform.rotation.x
            qy += transform.transform.rotation.y
            qz += transform.transform.rotation.z
        t.transform.translation.x = x / len(transforms)
        t.transform.translation.y = y / len(transforms)
        t.transform.translation.z = z / len(transforms)
        t.transform.rotation.x = qx / len(transforms)
        t.transform.rotation.y = qy / len(transforms)
        t.transform.rotation.z = qz / len(transforms)
        return t

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
