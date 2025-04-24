import rclpy
from rclpy.action import ActionServer


from rovr_interfaces.action import AutoDig
from rovr_interfaces.srv import SetPosition, SetPower
from rclpy.action.server import ServerGoalHandle, CancelResponse
from std_srvs.srv import Trigger

from rovr_control.node_util import AsyncNode


class AutoDigServer(AsyncNode):
    def __init__(self):
        super().__init__("auto_dig_server")
        self._action_server = ActionServer(
            self, AutoDig, "auto_dig", self.execute_callback, cancel_callback=self.cancel_callback
        )

        self.cli_lift_zero = self.create_client(Trigger, "lift/zero")
        self.cli_lift_bottom = self.create_client(Trigger, "lift/bottom")
        self.cli_lift_setPosition = self.create_client(SetPosition, "lift/setPosition")
        self.cli_lift_set_power = self.create_client(SetPower, "lift/setPower")
        self.cli_lift_stop = self.create_client(Trigger, "lift/stop")

        self.cli_digger_stop = self.create_client(Trigger, "digger/stop")
        self.cli_digger_setPower = self.create_client(SetPower, "digger/setPower")

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Starting Autonomous Digging Procedure!")
        result = AutoDig.Result()

        # Make sure the services are available
        if not self.cli_lift_setPosition.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Lift set position service not available")
            goal_handle.abort()
            return result
        if not self.cli_lift_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Lift stop service not available")
            goal_handle.abort()
            return result
        if not self.cli_lift_set_power.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Lift set power service not available")
            goal_handle.abort()
            return result
        if not self.cli_lift_zero.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Lift zero service not available")
            goal_handle.abort()
            return result
        if not self.cli_lift_bottom.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Lift bottom service not available")
            goal_handle.abort()
            return result
        if not self.cli_digger_setPower.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Digger set power service not available")
            goal_handle.abort()
            return result
        if not self.cli_digger_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Digger stop service not available")
            goal_handle.abort()
            return result

        # Start the digger belt
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Starting the digger belt")
            await self.cli_digger_setPower.call_async(SetPower.Request(power=goal_handle.request.digger_chain_power))

        # Lower the digger so that it is just above the ground (get to this position fast)
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Lowering the digger to the starting position")
            await self.cli_lift_setPosition.call_async(
                SetPosition.Request(position=goal_handle.request.lift_digging_start_position)
            )

        # Lower the digger into the ground slowly
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Lowering the digger into the ground")
            await self.cli_lift_bottom.call_async(Trigger.Request())

        # Stay at the lowest position for 5 seconds while digging
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Start of Auto Digging in Place")
            await self.async_sleep(5)
            self.get_logger().info("Done Digging in Place")

        # Stop digging
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Stopping the digger belt")
            await self.cli_digger_stop.call_async(Trigger.Request())

        # Raise the digger back up using the lift (get to this position fast)
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Raising the digger to the ending position")
            await self.cli_lift_setPosition.call_async(
                SetPosition.Request(position=goal_handle.request.lift_digging_end_position)
            )

        # Raise the digger the rest of the way slowly
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Raising the digger up to the top")
            await self.cli_lift_zero.call_async(Trigger.Request())

        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Autonomous Digging Procedure Complete!")
            goal_handle.succeed()
            return result
        else:
            self.get_logger().info("Goal was cancelled")
            goal_handle.canceled()
            return result

    def cancel_callback(self, cancel_request: ServerGoalHandle):
        """This method is called when the action is canceled."""
        super().cancel_callback(cancel_request)
        self.get_logger().info("Goal is cancelling")
        self.cli_digger_stop.call_async(Trigger.Request())
        self.cli_lift_stop.call_async(Trigger.Request())
        return CancelResponse.ACCEPT


def main(args=None) -> None:
    rclpy.init(args=args)

    action_server = AutoDigServer()
    rclpy.spin(action_server)

    action_server.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
