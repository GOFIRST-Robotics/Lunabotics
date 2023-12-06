from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import xml.etree.ElementTree as ET
import os

"""Make sure to turn on the camera using 
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py
or nothing here will work"""


class test_multiple_tags(Node):
    def __init__(self):
        super().__init__("test_multiple_tags")
        current_dir = os.getcwd()

        """Change this based on the field."""
        relative_path = "src/apriltag/apriltag/apriltag_location_nasa.urdf.xarco"
        # relative_path = "src/apriltag/apriltag/apriltag_location_ucf.urdf.xarco"

        self.file_path = os.path.join(current_dir, relative_path)

        #TODO: figure out how we distinguish between the tags based on camera used.
        self.transforms = self.create_subscription(AprilTagDetectionArray, "/tag_detections", self.sendTransform, 10)
        self.tf_broadcaster = TransformBroadcaster(self)


    def sendTransform(self, msg):
        if len(msg.detections) == 0:
            return
        
        self.makeTransforms(msg.detections)

        # While this is only being used for a start transform.
        # probably shouldnt kill itself if we want to keep publishing transforms
        return

    def makeTransforms(self, tags):
        transforms = []
        for tag in tags:
            t = TransformStamped()
            t.child_frame_id = "odom"
            t.header.frame_id = "map"
            t.header.stamp = self.get_clock().now().to_msg()

            id = tag.id
            tree = ET.parse(self.file_path)
            root = tree.getroot()

            link = root[id - 1] # assumes tag 1 = home 1, tag 2 = home 2, 3 = berm 1 etc.

            xyz_elements = link.findall(".//origin[@xyz]")
            xyz_values = [element.attrib["xyz"] for element in xyz_elements]
            xyz = xyz_values[0].split(" ")

            rpy_elements = link.findall(".//origin[@rpy]")
            rpy_values = [element.attrib["rpy"] for element in rpy_elements]
            rpy = rpy_values[0].split(" ")

            t.transform.translation.x = tag.pose.pose.pose.position.x - float(xyz[0])
            t.transform.translation.y = tag.pose.pose.pose.position.z - float(xyz[1])
            t.transform.translation.z = tag.pose.pose.pose.position.y - float(xyz[2])
            t.transform.rotation.x = tag.pose.pose.pose.orientation.x - float(rpy[0])
            t.transform.rotation.y = tag.pose.pose.pose.orientation.y - float(rpy[1])
            t.transform.rotation.z = tag.pose.pose.pose.orientation.z - float(rpy[2])

            transforms.append(t)

        t = TransformStamped()
        t.child_frame_id = "odom"
        t.header.frame_id = "map"
        t.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.averageTransforms(transforms, t))
    
    """Averages the transforms of the tags to get a more accurate transform"""
    # TODO: might be interesting to weigh the transforms based on the distance from the camera
    def averageTransforms(self, transforms, t):
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


        print(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        return t

def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = test_multiple_tags()
    node.get_logger().info("Initializing the Apriltag node!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
