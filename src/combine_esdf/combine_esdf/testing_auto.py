import rclpy
from rclpy.node import Node
from PIL import Image
import numpy as np
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)

# Provides a “navigation as a library” capability

#https://navigation.ros.org/commander_api/index.html
class test_auto(Node):
    def __init__(self):
        super().__init__('test_auto')
        print("he")


        self.DANGER_THRESHOLD = 100
        self.REAL_DANGER_THRESHOLD = 200

        self.nav2 = BasicNavigator()
        # self.nav2.waitUntilNav2Active()
        self.optimal_dig_location()



    def dig(self):
        optimal_dig_location = self.optimal_dig_location()
        if optimal_dig_location is None:
            return TaskResult.FAILURE

        for location in optimal_dig_location:
            self.nav2.goTo((location * RESOLUTION) + OFFSET_X, 2.47, 180)
            self.LOWERSKIMMMER()
            self.nav2.goTo((location * RESOLUTION) + OFFSET_X, 4.47, 180)

            self.nav2.goTo(DUMPZONE, 0)
            self.DUMP()

        
        return TaskResult.SUCCESS
        

    def optimal_dig_location(self):
        robot_width = 0.5
        # robot_height = 0.5
        dig_size = (0.5, 0.5) # Get the width and heighto f dig

        self.autonomous_field_type = "nasa"
        if self.autonomous_field_type == "top":
            dig = (8.14, 4.07)
            # self.autonomous_dig_location.pose.orientation.z = 0.0
        elif self.autonomous_field_type == "bottom":
            dig = (8.14, 4.07)
            # self.autonomous_dig_location.pose.orientation.z = 0.0
        elif self.autonomous_field_type == "nasa":
            dig = (1.3, 0.6)
            # self.autonomous_dig_location.pose.orientation.z = 0.0
        
        
        costmap = self.nav2.getGlobalCostmap()
        
        format = costmap.metadata
        data = np.array(costmap.data).reshape((format.size_x, format.size_y))

        if data is None or data.size == 0:
            return
        
        # this is almost definitely wrong. need to figure out where apriltag reset puts the origin.
        origin = (format.origin.position.x, format.origin.position.y)
        offset_x = int(abs(dig[0] - origin[0]) / format.resolution)
        offset_y = int(abs(dig[1] - origin[1]) / format.resolution)

        dig_zone_data = data[offset_x:offset_x + int(dig_size[0] / format.resolution), offset_y:offset_y + int(dig_size[1] / format.resolution)]
        if dig_zone_data is None or dig_zone_data.size == 0:
            return

        available_dig_spots = []
        robot_width_pixels = robot_width / format.resolution        # dig_zone_data[dig_zone_data <= 100] = 0 # Set this to whatever the "too dangerous" threshold is

        for i in range(dig_zone_data.shape[0]):
            if np.amax(dig_zone_data[int(i-robot_width_pixels/2):int(i+robot_width_pixels/2), :]) <= self.DANGER_THRESHOLD:
                available_dig_spots.append(i)
                i += robot_width_pixels
        
        if (len(available_dig_spots) == 0):
            self.DANGER_THRESHOLD += 5
            if self.DANGER_THRESHOLD > self.REAL_DANGER_THRESHOLD:
                self.get_logger().info("No available dig spots. Switch to Teleop por favor!")
                return None
            
        return available_dig_spots
    


def main(args=None):
    rclpy.init(args=args)

    test_auto_node = test_auto()

    rclpy.spin(test_auto_node)

    test_auto_node.destroy_node()
    rclpy.shutdown()