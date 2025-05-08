import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.action.server import ServerGoalHandle
from nav2_msgs.action import BackUp
from rovr_interfaces.action import AutoDig, AutoOffload, AutoDigNavOffload
from rovr_control.node_util import AsyncNode


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
        self._backup_client = ActionClient(self, BackUp, "backup_server")
        self._auto_offload_client = ActionClient(self, AutoOffload, "auto_offload")

        self.dig_in_progress = False
        self.backup_in_progress = False
        self.offload_in_progress = False

        self.get_logger().info("Auto Dig–Backup–Offload server ready")

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        result = AutoDigNavOffload.Result()

        # 1. Dig
        if not goal_handle.is_cancel_requested:
            if not await self._do_auto_dig(goal_handle):
                goal_handle.abort()
                return result

        # 2. Back up
        if not goal_handle.is_cancel_requested:
            if not await self._do_backup(goal_handle):
                goal_handle.abort()
                return result

        # 3. Offload
        if not goal_handle.is_cancel_requested:
            if not await self._do_auto_offload(goal_handle):
                goal_handle.abort()
                return result

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result

    async def _do_auto_dig(self, goal_handle):
        self.get_logger().info("→ Starting AutoDig")
        if not self._auto_dig_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("AutoDig server unavailable")
            return False

        self.dig_in_progress = True
        dig_goal = AutoDig.Goal(
            lift_digging_start_position=goal_handle.request.lift_digging_start_position,
            digger_chain_power=goal_handle.request.digger_chain_power,
        )
        send = await self._auto_dig_client.send_goal_async(dig_goal)
        if not send.accepted:
            self.get_logger().error("AutoDig rejected")
            self.dig_in_progress = False
            return False

        await send.get_result_async()
        self.dig_in_progress = False
        self.get_logger().info("→ AutoDig complete")
        return True

    async def _do_backup(self, goal_handle):
        dist = goal_handle.request.backward_distance
        speed = 0.25  # duty cycle
        timeout = 20.0  # seconds
        self.get_logger().info(f"→ Backing up {dist} m @ {speed} (duty cycle)")

        if not self._backup_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("BackUp server unavailable")
            return False

        self.backup_in_progress = True
        backup_goal = BackUp.Goal(
            backup_dist=-abs(dist),  # negative = backward
            backup_speed=speed,
            time_allowance=timeout,
            server_name="backup_server",
            server_timeout=10.0,
            disable_collision_checks=False,
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
        if result.status == result.Status.SUCCEEDED and result.result.error_code_id == 0:
            self.get_logger().info("→ BackUp succeeded")
            return True
        else:
            self.get_logger().error(f"BackUp failed: {result.result.error_msg}")
            return False

    async def _do_auto_offload(self, goal_handle):
        self.get_logger().info("→ Starting AutoOffload")
        if not self._auto_offload_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("AutoOffload server unavailable")
            return False

        self.offload_in_progress = True
        offload_goal = AutoOffload.Goal()
        send = await self._auto_offload_client.send_goal_async(offload_goal)
        if not send.accepted:
            self.get_logger().error("AutoOffload rejected")
            self.offload_in_progress = False
            return False

        await send.get_result_async()
        self.offload_in_progress = False
        self.get_logger().info("→ AutoOffload complete")
        return True

    def cancel_callback(self, cancel_request):
        super().cancel_callback(cancel_request)
        self.get_logger().info("→ Cancellation requested")
        if self.dig_in_progress:
            self.get_logger().info("Cancelling AutoDig")
        if self.backup_in_progress:
            self._backup_client.cancel_all_goals()  # cancel Nav2 backup
            self.get_logger().info("Cancelling BackUp")
        if self.offload_in_progress:
            self.get_logger().info("Cancelling AutoOffload")
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    server = AutoDigNavOffloadServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
