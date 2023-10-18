import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from sensor_msgs.msg import Image, float32
import cv2 
from cv_bridge import CvBridge
import math

CONVEYORSIZEX = .5 # width of the conveyor in meters
CONVEYORSIZEY = .5 # length of the conveyor in meters
CONVEYORTOCAM = .3 # distance from camera to top of conveyor in meters.
DISTANCETHRESH = 200 # how small should the distance between top and conveyor be before offload (in meters)?
POLLRATE = .5 # Wait time between each distance check (in seconds)
CONSECUTIVECYCLES = 4 # Make sure the reading is consistent
conveyor_height_topic = "/conveyor/height" # should be meters, if not, convert

class ros_check_load(Node):

    def __init__(self):
        super().__init__("check_load")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Bool, 'readyDump', 10)
        self.prior_checks = []
        depth_image_topic = '/camera/depth/image_rect_raw' # Switch to /depth/image_rect_raw when running on robot
        self.getConveyorHeight = self.create_subscription(float32, conveyor_height_topic, self.setHeight, 10)
        self.oneTimeSub = self.create_subscription(Image, depth_image_topic, self.setParamCallback, 1)
        self.subscriber = self.create_subscription(Image, depth_image_topic, self.depth_image_callback, 10)
        self.timer = self.create_timer(POLLRATE, self.publish_distance)
        self.depth_image = None
        
    def setHeight(self, msg):
        self.conveyor_height = msg.data
    
    def setParamCallback(self, msg):
        self.img_height = msg.height
        self.img_width = msg.width
        #center_x, center_y = width // 2, height // 2
        # self.start_x = max(0, center_x - width // 4)
        # self.end_x = min(width, center_x + width // 4)
        # self.start_y = max(0, center_y - height // 4)
        # self.end_y = min(height, center_y + (height // 4))
        self.destroy_subscription(self.oneTimeSub)
        
    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        

    def publish_distance(self):
        try:
            depth = self.depth_image

            # find the degrees of vision occupied by the conveyor belt
            perceptionChangeX = 2 * math.atan((.5 * CONVEYORSIZEX) / (self.conveyor_height + CONVEYORTOCAM)) * (180 / math.PI)
            perceptionChangeY = 2 * math.atan((.5 * CONVEYORSIZEY) / (self.conveyor_height + CONVEYORTOCAM)) * (180 / math.PI)

            percentFOVx = perceptionChangeX / 85.2 # compare the degrees of vision it would occupy to the FOV of the realsense
            percentFOVy = perceptionChangeY / 58.5

            change_x = (self.img_width * percentFOVx) / 2  # get the number of pixels that it's occupying. divide by 2 so it's a +- situation
            change_y = (self.img_height * percentFOVy) / 2
            
            center_x, center_y = self.img_width // 2, self.img_height // 2
            resized_img = depth[center_y - change_y:center_y + change_y, center_x - change_x:center_x + change_x]

            denoised_image = cv2.GaussianBlur(resized_img, (5, 5), 0)

            print(denoised_image.mean())
            
            if denoised_image.mean() <= DISTANCETHRESH + self.conveyor_height:
                
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