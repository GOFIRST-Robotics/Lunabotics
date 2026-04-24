import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rovr_interfaces.action import ReturnToCoordinate

# This is the server that will return the robot to a specified coordinate.
#  It will be used in the auto dig-nav-offload action to return the robot
# to the dig location after offloading.
from rclpy.action.server import ServerGoalHandle, CancelResponse
from action_msgs.msg import GoalStatus
from rovr_control.node_util import AsyncNode
from nav2_msgs.action import NavigateToPose
from rclpy.action.client import ClientGoalHandle
from nav_msgs.msg import OccupancyGrid
import math


class ReturnToCoordinateServer(AsyncNode):
    def __init__(self):
        super().__init__("return_to_coordinate_server")

        self._action_server = ActionServer(
            self,
            ReturnToCoordinate,
            "return_to_coordinate",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            # handle_accepted_callback is a function that is called when a goal is accepted,
            #  it is used to start the execution of the goal in a new thread.
        )

        self.backup_in_progress = False

        self.coord_return_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.coord_return_handle = ClientGoalHandle(None, None, None)

    def goal_callback(self, goal_request):
        self.get_logger().info(
            "Received goal request to return to coordinate ({}, {})".format(
                goal_request.x, goal_request.y
            )
        )
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info("Accepted goal to return to coordinate, starting execution")
        goal_handle.execute()

    def cancel_callback(self, goal_handle):

        super().cancel_callback(goal_handle)
        self.get_logger().info("Received request to cancel return to coordinate action")

        if self.coord_return_handle:
            if self.coord_return_handle.status == GoalStatus.STATUS_EXECUTING:
                self.get_logger().info("Cancelling return to coordinate action")
                self.coord_return_handle.cancel_goal_async()

            if not self.coord_return_handle.is_done():
                self.get_logger().info("Cancelling return to coordinate action")
                self.coord_return_handle.cancel_goal_async()

            if self.coord_return_handle.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Return to coordinate action cancelled successfully")
        return CancelResponse.ACCEPT

    def get_quat_from_euler(self, yaw):
        return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        # result = ReturnToCoordinate.Result() still need to define result
        # using boolean for now, can be expanded later if needed
        result = ReturnToCoordinate.Result()
        target_x = goal_handle.request.x_pos
        target_y = goal_handle.request.y_pos
        self.backup_in_progress = True

        if not self.coord_return_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Navigate to pose action server not available")
            goal_handle.abort()
            result.success = False
            return result

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y

        goal_quat = self.get_quat_from_euler(0.0)  # Facing forward
        goal_msg.pose.pose.orientation.x = goal_quat["x"]
        goal_msg.pose.pose.orientation.y = goal_quat["y"]
        goal_msg.pose.pose.orientation.z = goal_quat["z"]
        goal_msg.pose.pose.orientation.w = goal_quat["w"]

        intended_location = self.coord_return_client.send_goal_async(goal_msg)
        self.coord_return_handle = await intended_location

        if not self.coord_return_handle.accepted:
            self.get_logger().error("BackUp rejected")
            self.backup_in_progress = False
            goal_handle.abort()
            result.success = False
            return result

        self.get_logger().info("BackUp Goal Accepted")
        get_intended_location = self.coord_return_handle.get_result_async()
        intended_location_result = await get_intended_location

        intended_location_status = intended_location_result.status

        if intended_location_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("→ BackUp succeeded")
            goal_handle.succeed()
            self.backup_in_progress = False
            result.success = True
            return result

        elif intended_location_status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("BackUp Failed")
            goal_handle.canceled()
            self.backup_in_progress = False
            result.success = False
            return result

        return result

def main (args=None):
    rclpy.init(args=args)
    return_to_coordinate_server = ReturnToCoordinateServer()
    rclpy.spin(return_to_coordinate_server)

    try:
        rclpy.spin(return_to_coordinate_server)
    except KeyboardInterrupt:
        pass
    finally:
        return_to_coordinate_server.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()