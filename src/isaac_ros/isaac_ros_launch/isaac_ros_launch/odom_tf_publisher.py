import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image


class TFPublisher(Node):
    def __init__(self):
        super().__init__("odom_to_tf")

        self.most_recent_color_image = None
        self.most_recent_depth_image = None

        self.odom_tf_publisher = self.create_publisher(TFMessage, "tf", 10)
        self.odom_subscription = self.create_subscription(Odometry, "odom", self.callback, 10)
        self.camera_info_subscription = self.create_subscription(CameraInfo, "camera_info", self.camera_info_callback, 10)
        self.color_image_subscription = self.create_subscription(Image, "camera", self.color_image_callback, 10)
        self.depth_image_subscription = self.create_subscription(Image, "depth", self.depth_image_callback, 10)
        self.color_camera_info_publisher = self.create_publisher(CameraInfo, "color_camera_info", 10)
        self.depth_camera_info_publisher = self.create_publisher(CameraInfo, "depth_camera_info", 10)
        self.color_image_publisher = self.create_publisher(Image, "color_camera_image", 10)
        self.depth_image_publisher = self.create_publisher(Image, "depth_camera_image", 10)

    def callback(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()  # Set ROS time as the timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        tf_message = TFMessage(transforms=[t])
        self.odom_tf_publisher.publish(tf_message)

    def camera_info_callback(self, msg):
        if self.most_recent_color_image and msg.header.frame_id == "Master_ASM/zed2i_camera_link/camera":
            msg.header.frame_id = "zed2i_camera_link"
            msg.header.stamp = self.get_clock().now().to_msg()  # Set ROS time as the timestamp
            self.most_recent_color_image.header.stamp = msg.header.stamp
            self.color_camera_info_publisher.publish(msg)
            self.color_image_publisher.publish(self.most_recent_color_image)
        elif self.most_recent_depth_image and msg.header.frame_id == "Master_ASM/zed2i_camera_link/depthcamera":
            msg.header.frame_id = "zed2i_camera_link"
            msg.header.stamp = self.get_clock().now().to_msg()  # Set ROS time as the timestamp
            self.most_recent_depth_image.header.stamp = msg.header.stamp
            self.depth_camera_info_publisher.publish(msg)
            self.depth_image_publisher.publish(self.most_recent_depth_image)

    def color_image_callback(self, msg):
        msg.header.frame_id = "zed2i_camera_link"
        self.most_recent_color_image = msg

    def depth_image_callback(self, msg):
        msg.header.frame_id = "zed2i_camera_link"
        self.most_recent_depth_image = msg


def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TFPublisher()
    rclpy.spin(tf_publisher)
    tf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
