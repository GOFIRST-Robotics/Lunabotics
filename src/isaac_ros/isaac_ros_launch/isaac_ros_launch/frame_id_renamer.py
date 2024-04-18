import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class FrameIDRenamer(Node):
    def __init__(self):
        super().__init__("frame_id_renamer")

        self.camera_info_subscription = self.create_subscription(CameraInfo, "camera_info", self.camera_info_callback, 10)
        self.color_image_subscription = self.create_subscription(Image, "camera", self.color_image_callback, 10)
        self.depth_image_subscription = self.create_subscription(Image, "depth", self.depth_image_callback, 10)

        self.color_camera_info_publisher = self.create_publisher(CameraInfo, "color_camera_info", 10)
        self.depth_camera_info_publisher = self.create_publisher(CameraInfo, "depth_camera_info", 10)
        self.color_image_publisher = self.create_publisher(Image, "color_camera_image", 10)
        self.depth_image_publisher = self.create_publisher(Image, "depth_camera_image", 10)

    def camera_info_callback(self, msg):
        if msg.header.frame_id == "Master_ASM/zed2i_camera_link/camera":
            msg.header.frame_id = "zed2i_camera_link"
            self.color_camera_info_publisher.publish(msg)
        elif msg.header.frame_id == "Master_ASM/zed2i_camera_link/depthcamera":
            msg.header.frame_id = "zed2i_camera_link"
            self.depth_camera_info_publisher.publish(msg)

    def color_image_callback(self, msg):
        msg.header.frame_id = "zed2i_camera_link"
        self.color_image_publisher.publish(msg)

    def depth_image_callback(self, msg):
        msg.header.frame_id = "zed2i_camera_link"
        self.depth_image_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    frame_id_renamer = FrameIDRenamer()
    rclpy.spin(frame_id_renamer)
    frame_id_renamer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
