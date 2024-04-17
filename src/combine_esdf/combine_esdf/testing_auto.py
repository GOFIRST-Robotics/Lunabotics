import rclpy
from rclpy.node import Node
from PIL import Image
import numpy as np
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)

from tf2_msgs.msg import TFMessage

  # Provides a “navigation as a library” capability

class test_auto(Node):
    def __init__(self):
        super().__init__('test_auto')
        print("he")
        self.robot_location = None
        self.t = True
        self.nav2 = BasicNavigator()
        # self.nav2.waitUntilNav2Active()
        self.robot_location_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.robot_location_callback,
            10)

    def filter_transforms_by_frame_id(self, transforms, frame_id):
        return [t for t in transforms if t.header.frame_id == frame_id]

    def robot_location_callback(self, msg):
        if self.t:
            # self.robot_location = self.filter_transforms_by_frame_id(msg.transforms, "base_link")
            self.robot_location = msg.transforms
            print(self.robot_location)
            self.optimal_dig_location()
            self.t = False
            return
        

    def optimal_dig_location(self):
        # robot_width = 0.5
        # robot_height = 0.5
        # dig_size = (0.5, 0.5) # Get the width and heighto f dig

        # self.autonomous_field_type = "nasa"
        # if self.autonomous_field_type == "top":
        #     dig = (6.84, 3.57)
        #     # self.autonomous_dig_location.pose.orientation.z = 0.0
        # elif self.autonomous_field_type == "bottom":
        #     dig = (6.84, 1.0)
        #     # self.autonomous_dig_location.pose.orientation.z = 0.0
        # elif self.autonomous_field_type == "nasa":
        #     dig = (1.3, 0.6)
        #     # self.autonomous_dig_location.pose.orientation.z = 0.0
        
        
        costmap = self.nav2.getGlobalCostmap()
        format = costmap.metadata
        res = format.resolution
        data = np.array(costmap.data).reshape((format.size_x, format.size_y))

        if data is None or data.size == 0:
            return
        # this is almost definitely wrong. need to figure out where apriltag reset puts the origin.
        origin = (format.origin.position.x, format.origin.position.y)
        # offset_x = int((dig[0] - origin[0]) / format.resolution)
        # offset_y = int((dig[1] - origin[1]) / format.resolution)

        # dig_zone_data = data[offset_x:offset_x + int(dig_size[0] / format.resolution), offset_y:offset_y + int(dig_size[1] / format.resolution)]
        # if dig_zone_data is None or dig_zone_data.size == 0:
        #     return
        # # dig_zone_data[dig_zone_data <= 100] = 0 # Set this to whatever the "too dangerous" threshold is
        # robot_width_pixels = robot_width / format.resolution
        # for i in range(dig_zone_data.shape[0]):
        #     if np.amax(dig_zone_data[int(i-robot_width_pixels/2):int(i+robot_width_pixels/2), :]) <= 100:
        #         print("Found a good spot to dig at", dig[0], dig[1] + i * format.resolution)
        #         i+= robot_width_pixels

        # x = self.nav2.getOriginY(costmap)
        # print(format.origin.position.x, format.origin.position.y)
        image = Image.fromarray(data.astype('uint8'))

        # Save the image
        image.save('matrix_image.png')



def main(args=None):
    rclpy.init(args=args)

    test_auto_node = test_auto()

    rclpy.spin(test_auto_node)

    test_auto_node.destroy_node()
    rclpy.shutdown()