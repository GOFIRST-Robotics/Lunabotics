import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

YRESOLUTION = 480 # whatever y componenet (vertical) resolution the camera is
XRESOLUTION = 640 # same as above, but in the x direction
DISTANCETHRESH = .25 # how small should the distance between top and conveyor be before offload (in meters)?
POLLRATE = 1 # Wait time between each distance check (in seconds)
CONSECUTIVECYCLES = 4 # Make sure the reading is consistent

class ros_check_load(Node):

    def __init__(self):
        super().__init__("check_load")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Bool, 'readyDump', 10)
        self.prior_checks = []
        depth_image_topic = '/depth/image_rect_raw'
        self.subscriber = self.create_subscription(Image, depth_image_topic, self.depth_image_callback, 10)
        self.timer = self.create_timer(POLLRATE, self.publish_distance)
        self.depth_image = None
        
    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        

    def publish_distance(self):
        try:
            depth = self.depth_image
            if depth is None:
                return
            
            height, width = depth.shape[:2]
            center_x, center_y = width // 2, height // 2
            start_x = max(0, center_x - XRESOLUTION // 2)
            end_x = min(width, center_x + XRESOLUTION // 2)
            start_y = max(0, center_y - YRESOLUTION // 2)
            end_y = min(height, center_y + YRESOLUTION // 2)
            roi = depth[start_y:end_y, start_x:end_x]

            if roi.mean() <= DISTANCETHRESH:
                self.prior_checks.append(True)
            else:
                self.prior_checks.append(False)
                
            if len(self.prior_checks >= CONSECUTIVECYCLES):
                self.prior_checks.pop(0)
                if (False not in self.prior_checks):
                    msg = Bool()
                    msg.data = True
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