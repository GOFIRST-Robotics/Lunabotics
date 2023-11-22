
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

class Apriltag(Node):
    def __init__(self):
        super().__init__('apriltag')
        current_dir = os.getcwd()

        # swap comment based on the field.
        relative_path = "src/apriltag/apriltag/apriltag_location_nasa.urdf.xarco"
        # relative_path = "src/apriltag/apriltag/apriltag_location_ucf.urdf.xarco"
        
        self.file_path = os.path.join(current_dir, relative_path)
        self.transforms = self.create_subscription(AprilTagDetectionArray, "/tag_detections", self.sendTransform, 10)
        # self.transforms = self.create_subscription(AprilTagDetectionArray, "/tag_detections", self.sendTransform, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # self.timer = self.create_timer(0.1, self.test)
        
    """Delete this after stuff actually works."""
    def test(self):
        t = TransformStamped()
        t.child_frame_id = "odom"
        t.header.frame_id = 'map'
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x = float(4)
        self.tf_broadcaster.sendTransform(t)
        

    def printTransforms(self, msg):
        if len(msg.detections) == 0:
            return
        home = msg.detections[0]
        print(home.pose.pose.pose.position.x)
        print(home.pose.pose.pose.position.y)
        print(home.pose.pose.pose.position.z)

    """TODO: transform the tags to find the base_link and odom transforms"""
    def sendTransform(self, msg):
        if len(msg.detections) == 0:
            return
        
        # tag filtering. For now, the tag at (0,0) is id 3, the berm is id 1
        # Can filter through the tags, but still only uses the first one.
        # for tag in msg.detections:
            # if tag.id == 3:
            #     home = tag


        tag = msg.detections[0]
        t = TransformStamped()
        t.child_frame_id = "odom"
        t.header.frame_id = "map"
        t.header.stamp = self.get_clock().now().to_msg()
        self.makeTransforms(t, tag)
        
        # if home is not None:
        #     t = TransformStamped()
        #     t.child_frame_id = "odom"
        #     t.header.frame_id = "map"
        #     t.header.stamp = self.get_clock().now().to_msg()
        #     self.makeTransforms(t, home)

        # if berm is not None:
        #     m = TransformStamped()
        #     m.child_frame_id = "map"
        #     m.header.frame_id = 'other'
        #     m.header.stamp = self.get_clock().now().to_msg()
            
        #     self.makeTransforms(m, berm)
        # t.transform.translation.x = home.pose.pose.pose.position.x
        # t.transform.translation.y = home.pose.pose.pose.position.y
        # t.transform.translation.z = home.pose.pose.pose.position.z
        # t.transform.rotation.x = home.pose.pose.pose.orientation.x
        # t.transform.rotation.y = home.pose.pose.pose.orientation.y
        # t.transform.rotation.z = home.pose.pose.pose.orientation.z
        # self.tf_broadcaster.sendTransform(t)


        

    def makeTransforms(self, t, tag):
        id = tag.id
        tree = ET.parse(self.file_path)
        root = tree.getroot()
        
        link = root[id - 1] # assumes tag 1 = home 1, tag 2 = home 2, 3 = berm 1 etc.

        xyz_elements = link.findall(".//origin[@xyz]")
        xyz_values = [element.attrib["xyz"] for element in xyz_elements]
        xyz = xyz_values.split(" ")

        rpy_elements = link.findall(".//origin[@rpy]")
        rpy_values = [element.attrib["rpy"] for element in rpy_elements]
        rpy = rpy_values.split(" ")

        t.transform.translation.x = tag.pose.pose.pose.position.x - xyz[0]
        t.transform.translation.y = tag.pose.pose.pose.position.z - xyz[1]
        t.transform.translation.z = tag.pose.pose.pose.position.y - xyz[2]
        t.transform.rotation.x = tag.pose.pose.pose.orientation.x - rpy[0]
        t.transform.rotation.y = tag.pose.pose.pose.orientation.y - rpy[1]
        t.transform.rotation.z = tag.pose.pose.pose.orientation.z - rpy[2]
        self.tf_broadcaster.sendTransform(t)
    

def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = Apriltag()
    node.get_logger().info("Initializing the Apriltag subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()