import math

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle
from rovr_interfaces.action import CalibrateFieldCoordinates
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from nav2_msgs.action import Spin


class CalibrateFieldCoordinateServer(Node):
    def __init__(self):
        super().__init__("calibrate_field_action_server")
        self._action_server = ActionServer(
            self,
            CalibrateFieldCoordinates,
            "calibrate_field_coordinates",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )
        self.cli_set_apriltag_odometry = self.create_client(Trigger, "resetOdom")
        self.future_odom = Future()

        self.cli_spin = ActionClient(self, Spin, "spin")
        self.spin_handle = ClientGoalHandle(None, None, None)

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """This method rotates until we can see apriltag(s) and then sets the map -> odom tf."""
        result = CalibrateFieldCoordinates.Result()

        # Make sure the services are available
        if not self.cli_set_apriltag_odometry.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Apriltag odom service not available")
            goal_handle.abort()
            return result
        if not self.cli_spin.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 service not available")
            goal_handle.abort()
            return result

        # Spin around to find the apriltag
        spin_goal = Spin.Goal(target_yaw=2 * math.pi)
        self.spin_handle: ClientGoalHandle = await self.cli_spin.send_goal_async(spin_goal)
        future_spin: Future = self.spin_handle.get_result_async()
        self.future_odom = Future()

        while not future_spin.done():
            self.future_odom = self.cli_set_apriltag_odometry.call_async(Trigger.Request())
            if (await self.future_odom).success is True:
                await self.spin_handle.cancel_goal_async()

        if self.spin_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn("Failed to find apriltag")
            goal_handle.abort()
            return result

        if not self.cli_spin.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Apriltag odom service not available")
            goal_handle.abort()
            return result
        spin_goal = Spin.Goal(target_yaw=math.pi)
        self.spin_handle: ClientGoalHandle = await self.cli_spin.send_goal_async(spin_goal)
        await self.spin_handle.get_result_async()
        goal_handle.succeed()
        return result

    def cancel_callback(self, cancel_request: ServerGoalHandle):
        """This method is called when the action is canceled."""
        self.get_logger().info("Goal is cancelling")
        if self.spin_handle.status == GoalStatus.STATUS_EXECUTING:
            self.spin_handle.cancel_goal_async()
        if not self.future_odom.done():
            self.future_odom.cancel()
        return CancelResponse.ACCEPT


def main(args=None) -> None:
    rclpy.init(args=args)

    calibrate_field_action_server = CalibrateFieldCoordinateServer()
    rclpy.spin(calibrate_field_action_server)

    calibrate_field_action_server.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
