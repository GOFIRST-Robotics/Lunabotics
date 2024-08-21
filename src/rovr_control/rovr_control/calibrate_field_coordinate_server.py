import math

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle, CancelResponse
from nav2_msgs.action import Spin
from rovr_interfaces.action import CalibrateFieldCoordinates
from rovr_interfaces.srv import ResetOdom
from rclpy.node import Node
from rclpy.task import Future


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
        self.client_node = rclpy.create_node("calibrate_field_action_server_client")
        self.cli_set_apriltag_odometry = self.client_node.create_client(ResetOdom, "resetOdom")
        self.future_odom = Future()

        self.cli_spin = ActionClient(self.client_node, Spin, "spin")
        self.spin_handle = ClientGoalHandle(None, None, None)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """This method rotates until we can see apriltag(s) and then sets the map -> odom tf."""
        self.get_logger().info("Executing goal...")
        result = CalibrateFieldCoordinates.Result()
        self.get_logger().info("Beginning search for apriltags")

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
        spin_request = self.cli_spin.send_goal_async(spin_goal)
        rclpy.spin_until_future_complete(self.client_node, spin_request)
        self.spin_handle: ClientGoalHandle = spin_request.result()
        if not self.spin_handle.accepted:
            self.get_logger().error("Spin request was rejected")
            goal_handle.abort()
            return result
        future_spin: Future = self.spin_handle.get_result_async()
        self.future_odom = Future()

        while (
            not self.future_odom.cancelled()
            and not (self.future_odom.done() and self.future_odom.result().success is True)
            and not future_spin.done()
        ):
            self.future_odom = self.cli_set_apriltag_odometry.call_async(ResetOdom.Request())
            rclpy.spin_until_future_complete(self.client_node, self.future_odom)

        if self.spin_handle.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("Spin canceled")
            goal_handle.canceled()
            return result
        elif self.future_odom.done() and self.future_odom.result().success is True:
            self.get_logger().info("map -> odom TF published!")
            rclpy.spin_until_future_complete(self.client_node, self.spin_handle.cancel_goal_async())
        else:
            self.get_logger().error("Failed to find apriltag")
            goal_handle.abort()
            return result

        if not self.cli_spin.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Apriltag odom service not available")
            goal_handle.abort()
            return result
        spin_goal = Spin.Goal(target_yaw=math.pi)
        spin_request = self.cli_spin.send_goal_async(spin_goal)
        rclpy.spin_until_future_complete(self.client_node, spin_request)
        self.spin_handle: ClientGoalHandle = spin_request.result()
        if not self.spin_handle.accepted:
            self.get_logger().error("Spin request was rejected")
            goal_handle.abort()
            return result
        rclpy.spin_until_future_complete(self.client_node, self.spin_handle.get_result_async())
        if goal_handle.status == GoalStatus.STATUS_CANCELING:
            self.get_logger().error("Final spin to field Spin canceled")
            goal_handle.canceled()
            return result
        goal_handle.succeed()
        return result

    def cancel_callback(self, cancel_request):
        self.get_logger().info("Goal was canceled")
        if self.spin_handle.status == GoalStatus.STATUS_EXECUTING:
            self.spin_handle.cancel_goal_async()
        self.future_odom.cancel()
        return CancelResponse.ACCEPT


def main(args=None) -> None:
    rclpy.init(args=args)
    calibrate_field_action_server = CalibrateFieldCoordinateServer()
    rclpy.spin(calibrate_field_action_server)
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
