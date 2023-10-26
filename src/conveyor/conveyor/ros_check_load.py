import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import math
import time

# TODO: NEED TO UPDATE CONVEYOR SIZE, DISTANCE, DISTANCE THRESHOLD
# TODO: Should probably update pollrate/ consecutive cycles
# TODO: make sure hte topic names are all correct

CONVEYORSIZEY = 0.7112  # width of the conveyor in meters
CONVEYORSIZEX = 0.623  # length of the conveyor in meters
CONVEYORTOCAM = 0.3  # distance from camera to top of conveyor in meters.
DISTANCETHRESH = 200  # how small should the distance between top and conveyor be before offload (in meters)?
POLLRATE = 0.2  # Wait time between each distance check (in seconds)
CONSECUTIVECYCLES = 4  # Make sure the reading is consistent
conveyor_height_topic = "/conveyor/height"  # should be meters, as a displacement from the conveyors starting position, if not, convert


class ros_check_load(Node):
    def __init__(self):
        self.conveyor_height = .5
        self.img_height = 640
        self.img_width = 848
        super().__init__("check_load")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Bool, "readyDump", 10)
        self.prior_checks = []
        depth_image_topic = "/conveyor/camera/depth/image_rect_raw"  # The launch file should remap so the realsense publishes to this topic
        self.getConveyorHeight = self.create_subscription(Float32, conveyor_height_topic, self.setHeight, 10)
        self.oneTimeSub = self.create_subscription(Image, depth_image_topic, self.setParamCallback, 10)
        self.subscriber = self.create_subscription(Image, depth_image_topic, self.depth_image_callback, 10)
        self.timer = self.create_timer(POLLRATE, self.publish_distance)
        self.depth_image = None
        # self.lastMessage = int(time.time()*1000.0)
        

    def setHeight(self, msg):
        self.conveyor_height = msg.data

    def setParamCallback(self, msg):
        self.img_height = msg.height
        self.img_width = msg.width
        self.destroy_subscription(self.oneTimeSub)

    def depth_image_callback(self, msg):
        # self.lastMessage = int(time.time()*1000.0)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

    def publish_distance(self):
        if self.depth_image is None:
            pass
        # if self.lastMessage - int(time.time()*1000.0) > 5000:
        #     self.destroy_node()
        try:
            if self.depth_image is None:
                self.destroy_subscription(self.subscriber)
            depth = self.depth_image

            # find the degrees of vision occupied by the conveyor belt
            perceptionChangeX = (
                (2 * (math.atan((0.5 * CONVEYORSIZEX) / (self.conveyor_height + CONVEYORTOCAM)))) * (180 / math.pi)
            )
            perceptionChangeY = (
                (2 * (math.atan((0.5 * CONVEYORSIZEY) / (self.conveyor_height + CONVEYORTOCAM)))) * (180 / math.pi)
            )

            percentFOVx = min(perceptionChangeX / 86, 1)
            # compare the degrees of vision it would occupy to the FOV of the realsense
            percentFOVy = min(perceptionChangeY / 57, 1)

            change_x = (
                int(self.img_width * percentFOVx / 2)
            )   # get the number of pixels that it's occupying. divide by 2 so it's a +- situation
            change_y = int((self.img_height * percentFOVy) / 2)

            center_x, center_y = self.img_width // 2, self.img_height // 2
            denoised_image = cv2.GaussianBlur(depth, (5, 5), 0)
            resized_img = denoised_image[center_y - change_y : center_y + change_y, center_x - change_x : center_x + change_x]

            if resized_img.mean() <= DISTANCETHRESH + self.conveyor_height:
                self.prior_checks.append(True)
            else:
                self.prior_checks.append(False)

            if len(self.prior_checks) >= CONSECUTIVECYCLES:
                self.prior_checks.pop(0)
                msg = Bool()
                if False not in self.prior_checks:
                    msg.data = True
                else:
                    msg.data = False
                self.pub.publish(msg)
        except cv2.error as e:
            self.get_logger().fatal(f"Something critically wrong with camera: {e}")
            self.get_logger().fatal("Killing check_load node")
            self.destroy_node()
            return
        except Exception as e:
            if e == "NoneType" or e == "ZeroDivisionError":
                self.get_logger().fatal(f"Error: Realsense not publishing to topic: {e}")
            else:
                self.get_logger().fatal(f"Error: {e}")
            return


def main(args=None):
    """The main function."""
    rclpy.init(args=args)
    
    node = ros_check_load()
    node.get_logger().info("Initializing the Conveyor subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
