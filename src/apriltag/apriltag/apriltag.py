
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

"""Make sure to turn on the camera using 
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py"""

class Apriltag(Node):
    def __init__(self):
        super().__init__('apriltag')
        self.transforms = self.create_subscription(AprilTagDetectionArray, "/tag_detections", self.sendTransform, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # self.timer = self.create_timer(0.1, self.test)
        
    def test(self):
        t = TransformStamped()
        t.child_frame_id = "odom"
        t.header.frame_id = 'map'
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x = float(4)
        self.tf_broadcaster.sendTransform(t)
        
    """TODO: Make this work with multiple tags, or at least just filter and use the one we want.
    currently picks whatever /tag_detections indexes as the first tag"""
    def sendTransform(self, msg):
        if len(msg.detections) == 0:
            return
        t = TransformStamped()
        t.child_frame_id = "odom"
        t.header.frame_id = 'map'
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x = msg.detections[0].pose.pose.pose.position.x
        t.transform.translation.y = msg.detections[0].pose.pose.pose.position.y
        # t.transform.translation.z = msg.detections[0].pose.pose.pose.position.z
        t.transform.rotation.x = msg.detections[0].pose.pose.pose.orientation.x
        t.transform.rotation.y = msg.detections[0].pose.pose.pose.orientation.y
        t.transform.rotation.z = msg.detections[0].pose.pose.pose.orientation.z
        self.tf_broadcaster.sendTransform(t)
        
def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = Apriltag()
    node.get_logger().info("Initializing the Apriltag subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()