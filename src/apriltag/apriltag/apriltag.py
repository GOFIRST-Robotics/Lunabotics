
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

"""Make sure to turn on the camera using 
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py
or nothing here will work"""

class Apriltag(Node):
    def __init__(self):
        super().__init__('apriltag')
        self.transforms = self.create_subscription(AprilTagDetectionArray, "/tag_detections", self.sendTransform, 10)
        # self.transforms = self.create_subscription(AprilTagDetectionArray, "/tag_detections", self.sendTransform, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # self.timer = self.create_timer(0.1, self.test)
        
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
        home = None
        berm = None
        for tag in msg.detections:
            if tag.id == 3:
                home = tag
            elif tag.id == 1:
                berm = tag
        # home = msg.detections[0]    # comment this out if you want to use the loop
        if home is not None:
            t = TransformStamped()
            t.child_frame_id = "odom"
            t.header.frame_id = 'map'
            t.header.stamp = self.get_clock().now().to_msg()
            self.makeTransforms(t, home)

        if berm is not None:
            m = TransformStamped()
            m.child_frame_id = "map"
            m.header.frame_id = 'other'
            m.header.stamp = self.get_clock().now().to_msg()
            
            self.makeTransforms(m, berm)
        # t.transform.translation.x = home.pose.pose.pose.position.x
        # t.transform.translation.y = home.pose.pose.pose.position.y
        # # t.transform.translation.z = home.pose.pose.pose.position.z
        # t.transform.rotation.x = home.pose.pose.pose.orientation.x
        # t.transform.rotation.y = home.pose.pose.pose.orientation.y
        # t.transform.rotation.z = home.pose.pose.pose.orientation.z
        # self.tf_broadcaster.sendTransform(t)


        

    def makeTransforms(self, t, tag):
        t.transform.translation.x = tag.pose.pose.pose.position.x
        t.transform.translation.y = tag.pose.pose.pose.position.z
        t.transform.translation.z = tag.pose.pose.pose.position.y
        t.transform.rotation.x = tag.pose.pose.pose.orientation.x
        t.transform.rotation.y = tag.pose.pose.pose.orientation.y
        t.transform.rotation.z = tag.pose.pose.pose.orientation.z
        self.tf_broadcaster.sendTransform(t)
    

def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = Apriltag()
    node.get_logger().info("Initializing the Apriltag subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()