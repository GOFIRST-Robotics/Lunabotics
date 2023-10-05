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
        self.pub = self.create_publisher(Bool, 'readyDump', 10)
        self.prior_checks = []
        self.timer = self.create_timer(POLLRATE, self.publish_distance)
        depth_image_topic = '/camera/camera/depth/image_raw'
        self.depth_image = None
        subscriber = Node.create_subscription(self, Image, depth_image_topic, self.depth_image_callback, 10)

    def depth_image_callback(self, msg):
        bridge = CvBridge()
        self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def publish_distance(self):
        try:
           ##depth = frames.get_depth_frame() #not .get_depth_frane()
            average_tally = 0
            depth = self.depth_image
            for y in range(YRESOLUTION):
                for x in range(XRESOLUTION):
                    average_tally += depth.get_distance(x, y)
            msg = Bool()
            if len(self.prior_checks) >= CONSECUTIVECYCLES:
                self.prior_checks.pop(0)
            if average_tally / (XRESOLUTION * YRESOLUTION) <= DISTANCETHRESH:
                self.prior_checks.append(True)
            else:
                self.prior_checks.append(False)

            if False not in self.prior_checks and len(self.prior_checks) >= CONSECUTIVECYCLES:
                msg.data = True
                print("True")
            else:
                msg.data = False
                print("False")

            self.pub.publish(msg)

        except Exception as e:
            print(e)
            pass











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