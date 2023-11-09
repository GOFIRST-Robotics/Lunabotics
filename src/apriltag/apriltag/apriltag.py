
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class Apriltag(Node):
    def __init__(self):
        super().__init__('apriltag')
        self.transforms = self.create_subscription(TransformStamped, "tf2_msgs/TFMessage", self.sendTransform, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.test)
        
    def test(self):
        t = TransformStamped()
        t.child_frame_id = "odom"
        t.header.frame_id = 'map'
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x = float(4)
        self.tf_broadcaster.sendTransform(t)
        
    def sendTransform(self, msg):
        t = TransformStamped()
        t.child_frame_id = "odom"
        t.transform = msg[0]
        self.tf_broadcaster.sendTransform(t)
        
def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = Apriltag()
    node.get_logger().info("Initializing the Apriltag subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()