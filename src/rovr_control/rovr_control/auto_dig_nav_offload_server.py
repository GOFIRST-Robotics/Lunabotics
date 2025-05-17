import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.action.server import ServerGoalHandle
from nav2_msgs.action import BackUp
from rovr_interfaces.action import AutoDig, AutoOffload, AutoDigNavOffload
from rovr_control.node_util import AsyncNode
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from action_msgs.msg import GoalStatus
from rclpy.action.client import ClientGoalHandle


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
        self._backup_client = ActionClient(self, BackUp, "backup")
        self._auto_offload_client = ActionClient(self, AutoOffload, "auto_offload")

        # Status tracking
        self.dig_in_progress = False
        self.backup_in_progress = False
        self.offload_in_progress = False

        # Handle tracking
        self.dig_handle = ClientGoalHandle(None, None, None)
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
            await self._do_backup(goal_handle)

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

        if not goal_handle.is_cancel_requested:
            self.dig_in_progress = True
            dig_goal = AutoDig.Goal(
                lift_digging_start_position=goal_handle.request.lift_digging_start_position,
                digger_chain_power=goal_handle.request.digger_chain_power,
            )
            self.dig_handle = await self._auto_dig_client.send_goal_async(dig_goal)
            if not self.dig_handle.accepted:
                self.get_logger().error("AutoDig rejected")
                self.dig_in_progress = False
                return False

            await self.dig_handle.get_result_async()
            self.dig_in_progress = False
            self.get_logger().info("→ AutoDig complete")
            return True

    async def _do_backup(self, goal_handle):
        if not goal_handle.is_cancel_requested:
            dist = goal_handle.request.backward_distance
            speed = 0.5  # duty cycle
            timeout = 9.0  # seconds # TODO: Tune this value well!
            self.get_logger().info(f"→ Backing up {dist}m @ {speed} (duty cycle)")

            if not self._backup_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("BackUp server unavailable")
                return False

            target_point = Point()
            target_point.x = -abs(dist)  # negative x = backward
            target_point.y = 0.0
            target_point.z = 0.0

            self.backup_in_progress = True
            backup_goal = BackUp.Goal(
                speed=speed,
                target=target_point,
                time_allowance=Duration(sec=int(timeout)),
            )
            send = await self._backup_client.send_goal_async(backup_goal)
            if not send.accepted:
                self.get_logger().error("BackUp rejected")
                self.backup_in_progress = False
                return False

            # Wait for completion or cancel
            result_future = send.get_result_async()
            while not result_future.done():
                if goal_handle.is_cancel_requested:
                    self._backup_client.cancel_goal_async(send)  # ask Nav2 to stop
                    self.get_logger().info("BackUp canceled")
                    self.backup_in_progress = False
                    return False
                await self.async_sleep(0.1)

            result = await result_future
            self.backup_in_progress = False
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("→ BackUp succeeded")
                return True
            else:
                self.get_logger().error(f"BackUp failed: {result}")
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
