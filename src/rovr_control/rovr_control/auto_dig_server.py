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

        # Lower the digger so that it is just above the ground
        self.cli_lift_setPosition.call_async(
            SetPosition.Request(position=goal_handle.request.lift_digging_start_position)
        )

        # Start the digger belt
        await self.cli_digger_setPower.call_async(SetPower.Request(power=goal_handle.request.digger_belt_power))

        # Lower the digger towards the ground slowly
        if not self.running:
            self.get_logger().error("Digging buckets not spinning. Don't lower!")
            goal_handle.abort()
            return result
        await self.cli_lift_bottom.call_async(Trigger.Request())

        self.get_logger().info("Start of Auto Digging in Place")
        await self.async_sleep(5)  # Stay at the lowest position for 5 seconds while digging
        self.get_logger().info("Done Digging in Place")

        # Stop digging
        await self.cli_digger_stop.call_async(Trigger.Request())

        # Raise the digger back up using the lift
        await self.cli_lift_zero.call_async(Trigger.Reqest())

        self.get_logger().info("Autonomous Digging Procedure Complete!")
        goal_handle.succeed()
        return result

    def cancel_callback(self, cancel_request: ServerGoalHandle):
        """This method is called when the action is canceled."""
        self.get_logger().info("Goal is cancelling")
        if super().cancel_callback(cancel_request) == CancelResponse.ACCEPT:
            self.cli_digger_stop.call_async(Trigger.Request())
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
