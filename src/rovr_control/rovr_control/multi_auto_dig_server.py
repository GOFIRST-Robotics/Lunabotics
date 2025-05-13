import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.action.server import ServerGoalHandle
from nav2_msgs.action import BackUp, NavigateToPose
from rovr_interfaces.action import AutoDig, AutoOffload, MultiAutoDig
from rovr_control.node_util import AsyncNode
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from action_msgs.msg import GoalStatus
from rclpy.action.client import ClientGoalHandle
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time


class MultiAutoDigServer(AsyncNode):
    def __init__(self):
        super().__init__("multi_auto_dig_server")

        # Combined action server
        self._action_server = ActionServer(
            self,
            MultiAutoDig,
            "multi_auto_dig",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        # Sub-actions: dig, backup, offload, navigate
        self._auto_dig_client = ActionClient(self, AutoDig, "auto_dig")
        self._backup_client = ActionClient(self, BackUp, "backup")
        self._auto_offload_client = ActionClient(self, AutoOffload, "auto_offload")
        self._navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # TF buffer for pose tracking
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Status tracking
        self.dig_in_progress = False
        self.backup_in_progress = False
        self.offload_in_progress = False
        self.navigate_in_progress = False

        # Handle tracking
        self.dig_handle = ClientGoalHandle(None, None, None)
        self.offload_handle = ClientGoalHandle(None, None, None)

        # First dig pose (will be set during execution)
        self.first_dig_pose = None
        self.get_logger().info("Multi Auto Dig server ready")

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        result = MultiAutoDig.Result()
        request = goal_handle.request

        # Extract parameters
        num_digs = request.num_digs
        dig_offset = request.dig_offset

        # Save the initial pose as the first dig location
        if not await self._save_current_pose():
            self.get_logger().error("Failed to get initial pose")
            goal_handle.abort()
            return result

        self.get_logger().info(f"Performing {num_digs} digs with {dig_offset}m offset between positions")

        # Execute the sequence for each dig location
        for dig_index in range(num_digs):
            self.get_logger().info(f"Starting dig sequence {dig_index + 1}/{num_digs}")

            # If this is not the first dig, navigate to the next position
            if dig_index > 0 and not goal_handle.is_cancel_requested:
                if not await self._navigate_to_next_position(dig_index, dig_offset, goal_handle):
                    goal_handle.abort()
                    return result

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

            # Report progress
            feedback_msg = MultiAutoDig.Feedback()
            feedback_msg.current_dig = dig_index + 1
            feedback_msg.total_digs = num_digs
            goal_handle.publish_feedback(feedback_msg)

        # All digs completed successfully
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Multi Auto Dig Procedure Complete!")
            goal_handle.succeed()
            return result
        else:
            self.get_logger().info("Goal was cancelled")
            goal_handle.canceled()
            return result

    async def _save_current_pose(self):
        """Save the current robot pose from TF."""
        try:
            # Get the transform from odom to base_link
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Store the transform as our first dig pose
            self.first_dig_pose = transform

            self.get_logger().info(
                f"Saved initial dig pose at: "
                f"[{transform.transform.translation.x}, "
                f"{transform.transform.translation.y}]"
            )
            return True

        except TransformException as ex:
            self.get_logger().error(f"Could not get transform: {ex}")
            return False

    async def _navigate_to_next_position(self, dig_index, dig_offset, goal_handle):
        """Navigate to the next dig position based on the first pose and offset."""
        if not self._navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose server unavailable")
            return False

        # Calculate the next dig position (offset in y direction)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "odom"
        target_pose.header.stamp = self.get_clock().now().to_msg()

        # Use the first dig pose as reference
        target_pose.pose.position.x = self.first_dig_pose.transform.translation.x
        target_pose.pose.position.y = self.first_dig_pose.transform.translation.y - (dig_index * dig_offset)
        target_pose.pose.position.z = 0.0

        # Keep the same orientation as the first dig
        target_pose.pose.orientation = Quaternion(
            x=self.first_dig_pose.transform.rotation.x,
            y=self.first_dig_pose.transform.rotation.y,
            z=self.first_dig_pose.transform.rotation.z,
            w=self.first_dig_pose.transform.rotation.w,
        )

        self.get_logger().info(
            f"Navigating to next dig position: " f"[{target_pose.pose.position.x}, {target_pose.pose.position.y}]"
        )

        self.navigate_in_progress = True
        navigate_goal = NavigateToPose.Goal(pose=target_pose)

        send = await self._navigate_client.send_goal_async(navigate_goal)
        if not send.accepted:
            self.get_logger().error("NavigateToPose goal rejected")
            self.navigate_in_progress = False
            return False

        # Wait for result with cancel check
        result_future = send.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                await self._navigate_client.cancel_goal_async(send)
                self.get_logger().info("Navigation canceled")
                self.navigate_in_progress = False
                return False
            await self.async_sleep(0.1)

        result = await result_future
        self.navigate_in_progress = False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation to next dig position succeeded")
            return True
        else:
            self.get_logger().error(f"Navigation failed: {result.status}")
            return False

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
            timeout = 9.0  # seconds
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
                    await self._backup_client.cancel_goal_async(send)  # ask Nav2 to stop
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
                self.get_logger().error(f"BackUp failed: {result.status}")
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
        if self.navigate_in_progress:
            self.get_logger().info("Cancelling Navigation")
            self._navigate_client.cancel_all_goals()  # cancel Nav2 gotopose

        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    server = MultiAutoDigServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
