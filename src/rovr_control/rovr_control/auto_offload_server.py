import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, ServerGoalHandle, GoalStatus
from std_msgs.msg import Bool

from rovr_interfaces.action import AutoOffload
from rovr_interfaces.srv import Drive, Stop, SetPosition, SetPower

from rovr_control.node_util import AsyncNode


class AutoOffloadServer(AsyncNode):
    def __init__(self):
        super().__init__("auto_offload_server")
        self._action_server = ActionServer(
            self,
            AutoOffload,
            "auto_offload",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        # TODO: This should not be needed anymore after ticket #257 is implemented!
        self.skimmer_goal_subscription = self.create_subscription(
            Bool, "/skimmer/goal_reached", self.skimmer_goal_callback, 10
        )

        self.cli_drivetrain_drive = self.create_client(Drive, "drivetrain/drive")
        self.cli_drivetrain_stop = self.create_client(Stop, "drivetrain/stop")
        # TODO: This should not be updated to used #257
        self.cli_lift_setPosition = self.create_client(SetPosition, "lift/setPosition")
        self.cli_lift_stop = self.create_client(Stop, "lift/stop")

        self.cli_skimmer_setPower = self.create_client(SetPower, "skimmer/setPower")
        self.cli_skimmer_stop = self.create_client(Stop, "skimmer/stop")

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """This method lays out the procedure for autonomously offloading!"""
        self.get_logger().info("Starting Autonomous Offload Procedure!")
        result = AutoOffload.Result()

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
        if not self.cli_skimmer_setPower.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Skimmer set power service not available")
            goal_handle.abort()
            return result
        if not self.cli_skimmer_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Skimmer stop service not available")
            goal_handle.abort()
            return result

        # Drive backward into the berm zone
        self.cli_drivetrain_drive.call_async(Drive.Request(forward_power=-0.25, turning_power=0.0))

        self.get_logger().info("Auto Driving")
        # drive for 10 seconds
        await self.async_sleep(10)  # Allows for task to be canceled
        await self.cli_drivetrain_stop.call_async(Stop.Request())

        # Raise up the skimmer in preparation for dumping
        self.get_logger().info("Raising the Lift!")
        self.cli_lift_setPosition.call_async(SetPosition.Request(position=goal_handle.request.lift_dumping_position))
        # Wait for the lift goal to be reached
        await self.skimmer_sleep()
        if goal_handle.status != GoalStatus.STATUS_CANCELING:
            self.get_logger().info("Cancelling")
            return

        self.get_logger().info("Offloading")
        self.cli_skimmer_setPower.call_async(SetPower.Request(power=goal_handle.request.skimmer_belt_power))
        # sleep for the amount of time it takes to offload
        time_to_offload = 1.0 / abs(goal_handle.request.skimmer_belt_power)
        await self.async_sleep(time_to_offload)  # Allows for task to be canceled

        await self.cli_skimmer_stop.call_async(Stop.Request())  # Stop the skimmer belt
        self.get_logger().info("Autonomous Offload Procedure Complete!")
        goal_handle.succeed()
        return result

    def cancel_callback(self, cancel_request: ServerGoalHandle):
        """This method is called when the action is canceled."""
        self.get_logger().info("Goal is cancelling")
        # If lift is raising stop it
        if not self.skimmer_goal_reached.done():
            self.cli_drivetrain_stop.call_async(Stop.Request())
        if super().cancel_callback(cancel_request) == CancelResponse.ACCEPT:
            self.cli_drivetrain_stop.call_async(Stop.Request())
            self.cli_skimmer_stop.call_async(Stop.Request())
        return CancelResponse.ACCEPT


def main(args=None) -> None:
    rclpy.init(args=args)
    action_server = AutoOffloadServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
