import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.action.server import ServerGoalHandle
from nav2_msgs.action import NavigateToPose
from rovr_interfaces.action import AutoDig, AutoOffload, AutoDigNavOffload
from rovr_control.node_util import AsyncNode
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from action_msgs.msg import GoalStatus
from rclpy.action.client import ClientGoalHandle
import math


class AutoDigNavOffloadServer(AsyncNode):
    def __init__(self):
        super().__init__("auto_dig_nav_offload_server")

        # Combined action server
        self._action_server = ActionServer(
            self,
            AutoDigNavOffload,
            "auto_dig_nav_offload",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        # Sub‑actions: dig, backup, offload
        self._auto_dig_client = ActionClient(self, AutoDig, "auto_dig")
        self._backup_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._auto_offload_client = ActionClient(self, AutoOffload, "auto_offload")

        # Status tracking
        self.dig_in_progress = False
        self.backup_in_progress = False
        self.offload_in_progress = False

        # Handle tracking
        self.dig_handle = ClientGoalHandle(None, None, None)
        self.backup_handle = ClientGoalHandle(None, None, None)
        self.offload_handle = ClientGoalHandle(None, None, None)

        self.get_logger().info("Auto Dig-Backup-Offload server ready")

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        result = AutoDigNavOffload.Result()

        # 1. Dig
        if not goal_handle.is_cancel_requested:
            if not await self._do_auto_dig(goal_handle):
                goal_handle.abort()
                return result

        # 2. Back up
        if not goal_handle.is_cancel_requested:
            # Don't fail the whole action if backup fails
            await self._do_nav_backup(goal_handle)

        # 3. Offload
        if not goal_handle.is_cancel_requested:
            if not await self._do_auto_offload(goal_handle):
                goal_handle.abort()
                return result

        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Auto Dig Nav Offload Procedure Complete!")
            goal_handle.succeed()
            return result
        else:
            self.get_logger().info("Goal was cancelled")
            goal_handle.canceled()
            return result

    async def _do_auto_dig(self, goal_handle):
        self.get_logger().info("→ Starting AutoDig")
        if not self._auto_dig_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("AutoDig server unavailable")
            return False

        self.get_logger().info("AutoDig server available")

        if not goal_handle.is_cancel_requested:
            self.get_logger().info("start dig")

            self.dig_in_progress = True
            dig_goal = AutoDig.Goal(
                tilt_digging_start_position=goal_handle.request.tilt_digging_start_position,
                digger_chain_power=goal_handle.request.digger_chain_power,
            )
            self.dig_handle = await self._auto_dig_client.send_goal_async(dig_goal)
            if not self.dig_handle.accepted:
                self.get_logger().error("AutoDig rejected")
                self.dig_in_progress = False
                return False
            self.get_logger().info("AutoDig Goal Accepted")

            await self.dig_handle.get_result_async()
            self.dig_in_progress = False
            self.get_logger().info("→ AutoDig complete")
            return True

        return False
    
    def get_quat_from_euler(yaw):
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }
    
    async def _do_nav_backup(self, goal_handle):
        if not goal_handle.is_cancel_requested:
            if not self._backup_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error("Backup navigation service not available")
                goal_handle.abort()
                return False
            
            self.backup_in_progress = True

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = 1.0 # TODO: Update with real coordinate
            goal_msg.pose.pose.position.y = 0.0 # TODO: Update with real coordinate

            goal_quat = self.get_quat_from_euler(0.0)  # Facing forward
            goal_msg.pose.pose.orientation.x = goal_quat['x']
            goal_msg.pose.pose.orientation.y = goal_quat['y']
            goal_msg.pose.pose.orientation.z = goal_quat['z']
            goal_msg.pose.pose.orientation.w = goal_quat['w']

            self.get_logger().info("Sending backup goal")
            send_goal_future = self._backup_client.send_goal_async(goal_msg)
            send_goal_handle = await send_goal_future

            if not send_goal_handle.accepted:
                self.get_logger().error("BackUp rejected")
                self.backup_in_progress = False
                return False
            
            self.get_logger().info("BackUp Goal Accepted")
            get_result_future = send_goal_handle.get_result_async()
            result = await get_result_future

            status = result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("→ BackUp succeeded")
                self.backup_in_progress = False
                return True
            else:
                self.get_logger().error(f"BackUp failed: {result}")
                self.backup_in_progress = False
                return False

        return False

    async def _do_auto_offload(self, goal_handle):
        self.get_logger().info("→ Starting AutoOffload")
        if not self._auto_offload_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("AutoOffload server unavailable")
            return False

        if not goal_handle.is_cancel_requested:
            self.offload_in_progress = True
            offload_goal = AutoOffload.Goal()
            self.offload_handle = await self._auto_offload_client.send_goal_async(offload_goal)
            if not self.offload_handle.accepted:
                self.get_logger().error("AutoOffload rejected")
                self.offload_in_progress = False
                return False

            await self.offload_handle.get_result_async()
            self.offload_in_progress = False
            self.get_logger().info("→ AutoOffload complete")
            return True

    def cancel_callback(self, cancel_request):
        super().cancel_callback(cancel_request)
        self.get_logger().info("→ Cancellation requested")
        if self.dig_in_progress:
            self.get_logger().info("Cancelling AutoDig")
            self.dig_handle.cancel_goal_async()  # cancel AutoDig
        if self.backup_in_progress:
            self.get_logger().info("Cancelling BackUp")
            self._backup_client.cancel_all_goals()  # cancel Nav2 backup
        if self.offload_in_progress:
            self.get_logger().info("Cancelling AutoOffload")
            self.offload_handle.cancel_goal_async()  # cancel AutoOffload
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    server = AutoDigNavOffloadServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
