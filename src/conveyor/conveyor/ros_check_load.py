import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from sensor_msgs.msg import Image
import cv2 
from cv_bridge import CvBridge


DISTANCETHRESH = 200 # how small should the distance between top and conveyor be before offload (in meters)?
POLLRATE = .5 # Wait time between each distance check (in seconds)
CONSECUTIVECYCLES = 4 # Make sure the reading is consistent

class ros_check_load(Node):

    def __init__(self):
        super().__init__("check_load")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Bool, 'readyDump', 10)
        self.prior_checks = []
        depth_image_topic = '/camera/depth/image_rect_raw' # Switch to /depth/image_rect_raw when running on robot
        
        self.oneTimeSub = self.create_subscription(Image, depth_image_topic, self.setParamCallback, 1)
        self.subscriber = self.create_subscription(Image, depth_image_topic, self.depth_image_callback, 10)
        self.timer = self.create_timer(POLLRATE, self.publish_distance)
        self.depth_image = None
        
    def setParamCallback(self, msg):
        height = msg.height
        width = msg.width
        center_x, center_y = width // 2, height // 2
        self.start_x = max(0, center_x - width // 4)
        self.end_x = min(width, center_x + width // 4)
        self.start_y = max(0, center_y - height // 4)
        self.end_y = min(height, center_y + (height // 4))
        self.destroy_subscription(self.oneTimeSub)
        
    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        

    def publish_distance(self):
        try:
            depth = self.depth_image
            resized_img = depth[self.start_y:self.end_y, self.start_x:self.end_x]

            denoised_image = cv2.GaussianBlur(resized_img, (5, 5), 0)

            print(denoised_image.mean())
            
            if denoised_image.mean() <= DISTANCETHRESH:
                
                self.prior_checks.append(True)
            else:
                self.prior_checks.append(False)
                
            if (len(self.prior_checks) >= CONSECUTIVECYCLES):
                self.prior_checks.pop(0)
                msg = Bool()
                if (False not in self.prior_checks):
                    msg.data = True
                else:
                    msg.data = False
                self.pub.publish(msg)
        except Exception as e:
            print(e)
            return

def main(args=None):
    """The main function."""
    rclpy.init(args=args)
    print("Initializing the Conveyor subsystem!")

    node = ros_check_load()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()