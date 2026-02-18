import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rovr_control.costmap_2d import PyCostmap2D
from rovr_interfaces.srv import DigLocation
from rovr_interfaces.action import GoToDigLocation
from scipy.spatial.transform import Rotation as R
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
import math
from geometry_msgs.msg import PolygonStamped, PoseStamped


class DigLocationFinder(Node):
    def __init__(self):
        super().__init__("dig_location_server")
        self._action_server = ActionServer(
            self,
            GoToDigLocation,  # Empty action message
            "go_to_dig_location",
            self.drive_to_dig_location,
            # cancel_callback=self.drive_to_dig_location,
            # TODO: Make a cancel callback that actually cancels all running
            # futures please
        )
        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_costmap_global_srv = self.create_client(
            GetCostmap, "global_costmap/get_costmap"
        )
        self.srv = self.create_service(
            DigLocation, "find_dig_location", self.find_dig_location_callback
        )
        self.footprint_sub = self.create_subscription(
            PolygonStamped,
            "/local_costmap/published_footprint",
            self.get_robot_footprint,
            10,
        )
        self.footprint = (1.2, 0.75)
        self.absolute_max_dig_cost = self.declare_parameter(
            "absolute_max_dig_cost", 200
        ).value
        self.max_dig_cost = self.declare_parameter("max_dig_cost", 100).value
        self.all_dig_locations = (
            self.declare_parameter(
                "all_dig_locations",
                [
                    0.6,
                    0.37,
                    0.6,
                    1.1,
                    0.6,
                    1.83,
                    1.8,
                    0.37,
                    1.8,
                    1.1,
                    1.8,
                    1.83,
                    3.0,
                    0.37,
                    3.0,
                    1.1,
                    3.0,
                    1.83,
                    0.6,
                    2.5,
                    1.8,
                    2.5,
                    3.0,
                    2.5,
                ],
            ).value
        )  # If you default to an empty list things break (it thinks its a byte array)

        # ROS doesn't like nested lists, so the config file has to be
        # flattened. This unflattens that list
        self.all_dig_locations = [
            (self.all_dig_locations[i], self.all_dig_locations[i + 1])
            for i in range(0, len(self.all_dig_locations), 2)
        ]
        self.potential_dig_locations = self.all_dig_locations.copy()

    async def drive_to_dig_location(self, goal_handle):
        result = GoToDigLocation.Result()

        goal_pose_xy = self.getDigLocation()
        if goal_pose_xy is None:
            goal_handle.abort()
            self.get_logger().warn("goal_pose_xy is None")
            return result

        nav_goal = self.get_goal_pose(goal_pose_xy[0], goal_pose_xy[1], math.pi)

        send_goal_future = self.nav2_client.send_goal_async(nav_goal)
        goal_response = await send_goal_future
        if not goal_response.accepted:
            self.get_logger().error("Goal rejected")
            goal_handle.abort()
            return result

        result_future = goal_response.get_result_async()

        await result_future

        # result = result_future.result()
        if result and result.status == 4:  # STATUS_SUCCEEDED (4)
            self.get_logger().info("Navigation succeeded!")
            return result
        else:
            self.get_logger().error("Navigation failed!")
            return result

    # yaw in rads.
    # yaw = 0 at x axis, positive is counter clockwise
    def get_goal_pose(self, x, y, yaw):
        # if not self.client.wait_for_server(timeout_sec=2):
        #     self.get_logger().error("Action server not available")
        #     return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        # goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        quat = R.from_euler(
            "xyz", [float(0), float(0), float(yaw)], degrees=False
        ).as_quat()

        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]
        return goal_msg

    # ignore how unbelievably scuffed this is.
    def get_robot_footprint(self, msg):
        poly = msg.polygon
        points = poly.points

        x = math.sqrt(
            (points[0].x - points[1].x) ** 2 + (points[0].y - points[1].y) ** 2
        )
        y = math.sqrt(
            (points[1].x - points[2].x) ** 2 + (points[1].y - points[2].y) ** 2
        )

        width = max(x, y)
        height = min(x, y)
        self.footprint = (width, height)
        self.destroy_subscription(self.footprint_sub)

    def find_dig_location_callback(self, request, response):
        coords = self.getDigLocation()
        if coords is None:
            response.success = False
            return response

        response.success = True
        response.x = coords[0]
        response.y = coords[1]
        return response

    def updatePotentialDigLocations(self):
        try:
            costmap = PyCostmap2D(self.getGlobalCostmap())
            robot_width, robot_height = (0.5, 0.5)
            for location in self.potential_dig_locations:
                # dig_cost = maximum cost of the cells that the robot will dig
                dig_cost = costmap.getDigCost(
                    location[0], location[1], robot_width, robot_height
                )
                if dig_cost >= self.max_dig_cost:
                    self.potential_dig_locations.remove(location)

        except Exception as e:
            self.get_logger().error(f"Error in updatePotentialDigLocations {e}")

    def getGlobalCostmap(self) -> OccupancyGrid:
        """Get the global costmap."""
        while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info("Get global costmaps service not available, waiting...")
        req = GetCostmap.Request()
        future = self.get_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.error("Get global costmap request failed!")
            return None

        return result.map

    def getDigLocation(self):
        # self.updatePotentialDigLocations()
        # If there are no potential dig locations, reset the potential dig locations
        # and increase the max dig cost if the max dig cost is > absolute max
        # dig cost, return None
        if len(self.potential_dig_locations) == 0:
            self.potential_dig_locations = self.all_dig_locations.copy()
            self.max_dig_cost += 10
            if self.max_dig_cost >= self.absolute_max_dig_cost:
                return None
            return self.getDigLocation()
        return self.potential_dig_locations[0]


def main(args=None):
    rclpy.init(args=args)
    dig_location_finder = DigLocationFinder()
    rclpy.spin(dig_location_finder)
    dig_location_finder.destroy_node()
    rclpy.shutdown()
