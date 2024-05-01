import rclpy
from rclpy.node import Node
from PIL import Image
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from scipy.spatial.transform import Rotation as R


from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)

# Provides a “navigation as a library” capability

#https://navigation.ros.org/commander_api/index.html

# driveOnHeading(dist=0.15, speed=0.025, time_allowance=10)


def create_pose_stamped(x, y, yaw):
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.pose.position.x = x
    pose_stamped_msg.pose.position.y = y
    pose_stamped_msg.pose.orientation = R.from_euler("z", yaw, degrees=True).as_quat()
    return pose_stamped_msg


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
            self.nav2.goTo((location) + OFFSET_X, 2.47, 180)
            self.LOWERSKIMMMER()
            self.nav2.driveOnHeading(dist=2.57, speed = 0.025, time_allowance=10)

            self.nav2.goTo(DUMPZONE, 0)
            self.dump()

        
        return TaskResult.SUCCESS
    
    def dump(self):
        self.nav2.goToPose()
        

    def optimal_dig_location(self):
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
            if self.nav2.lineCost(-8.14 + i * .10, -8.14 + i*.10, 2.57, 0, .1) <= self.DANGER_THRESHOLD:
                available_dig_spots.append((-8.14 + i * .10, 2.57))
                i += robot_width / .10
        
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