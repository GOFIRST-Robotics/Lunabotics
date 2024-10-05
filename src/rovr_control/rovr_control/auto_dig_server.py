import rclpy
from rclpy.action import ActionServer

from std_msgs.msg import Bool

from rovr_interfaces.action import AutoDig
from rovr_interfaces.srv import Drive, Stop, SetPosition, SetPower
from rclpy.action.server import ServerGoalHandle, CancelResponse

from rovr_control.node_util import AsyncNode


class AutoDigServer(AsyncNode):
    def __init__(self):
        super().__init__("auto_dig_server")
        self._action_server = ActionServer(
            self, AutoDig, "auto_dig", self.execute_callback, cancel_callback=self.cancel_callback
        )

        # TODO: This should not be needed anymore after ticket #257 is implemented!
        self.digger_goal_subscription = self.create_subscription(
            Bool, "/digger/goal_reached", self.digger_goal_callback, 10
        )

        self.cli_drivetrain_drive = self.create_client(Drive, "drivetrain/drive")
        self.cli_drivetrain_stop = self.create_client(Stop, "drivetrain/stop")

        self.cli_lift_zero = self.create_client(Stop, "lift/zero")
        self.cli_lift_setPosition = self.create_client(SetPosition, "lift/setPosition")
        self.cli_lift_set_power = self.create_client(SetPower, "lift/setPower")
        self.cli_lift_stop = self.create_client(Stop, "lift/stop")

        self.cli_digger_stop = self.create_client(Stop, "digger/stop")
        self.cli_digger_setPower = self.create_client(SetPower, "digger/setPower")

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Starting Autonomous Digging Procedure!")
        result = AutoDig.Result()

        # Make sure the services are available
        if not self.cli_drivetrain_drive.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Drivetrain drive service not available")
            goal_handle.abort()
            return result
        if not self.cli_drivetrain_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Drivetrain stop service not available")
            goal_handle.abort()
            return result
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
        if not self.cli_digger_setPower.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("digger set power service not available")
            goal_handle.abort()
            return result
        if not self.cli_digger_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("digger stop service not available")
            goal_handle.abort()
            return result

        # Zero the digger
        self.cli_lift_zero.call_async(Stop.Request())
        # Wait for the lift goal to be reached
        await self.digger_sleep()

        # Lower the digger onto the ground
        self.cli_lift_setPosition.call_async(
            SetPosition.Request(position=goal_handle.request.lift_digging_start_position)
        )
        # Wait for the lift goal to be reached
        await self.digger_sleep()

        # Start the digger belt
        self.cli_digger_setPower.call_async(SetPower.Request(power=goal_handle.request.digger_belt_power))

        # Drive forward while digging
        start_time = self.get_clock().now().nanoseconds
        elapsed = self.get_clock().now().nanoseconds - start_time
        # accelerate for 2 seconds
        # TODO: completing ticket #298 can replace this while loop with a motor ramp up service
        while elapsed < 2e9:
            self.cli_lift_set_power.call_async(SetPower.Request(power=-0.05e-9 * (elapsed)))
            self.cli_drivetrain_drive.call_async(Drive.Request(forward_power=0.25e-9 * (elapsed), turning_power=0.0))
            self.get_logger().info("Accelerating lift and drive train")
            elapsed = self.get_clock().now().nanoseconds - start_time
            await self.async_sleep(0.1)  # Allows for task to be canceled

        self.get_logger().info("Auto Driving")
        await self.async_sleep(12)  # Allows for task to be canceled
        self.get_logger().info("Done Driving")

        # Stop driving and skimming
        await self.cli_drivetrain_stop.call_async(Stop.Request())
        await self.cli_digger_stop.call_async(Stop.Request())

        self.cli_lift_setPosition.call_async(SetPosition.Request(position=goal_handle.request.lift_dumping_position))
        # Wait for the lift goal to be reached
        await self.digger_sleep()

        self.get_logger().info("Autonomous Digging Procedure Complete!")
        goal_handle.succeed()
        return result

    def cancel_callback(self, cancel_request: ServerGoalHandle):
        """This method is called when the action is canceled."""
        self.get_logger().info("Goal is cancelling")
        if not self.digger_goal_reached.done():
            self.cli_drivetrain_stop.call_async(Stop.Request())
        if super().cancel_callback(cancel_request) == CancelResponse.ACCEPT:
            self.cli_drivetrain_stop.call_async(Stop.Request())
            self.cli_digger_stop.call_async(Stop.Request())
        return CancelResponse.ACCEPT


def main(args=None) -> None:
    rclpy.init(args=args)
    action_server = AutoDigServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
