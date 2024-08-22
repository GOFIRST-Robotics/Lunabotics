import rclpy
from rclpy.action import ActionServer
from rclpy.client import Future
from rclpy.node import Node

from rovr_interfaces.action import AutoDig
from rovr_interfaces.srv import Drive, Stop, SetPosition, SetPower

import time


class AutoDigServer(Node):
    def __init__(self):
        super().__init__("auto_dig_server")
        self._action_server = ActionServer(
            self, AutoDig, "auto_dig", self.execute_callback, cancel_callback=self.cancel_callback
        )

        self.declare_parameter("lift_dumping_position", -1000)  # Measured in encoder counts
        self.declare_parameter("lift_digging_start_position", -3050)  # Measured in encoder counts
        self.declare_parameter("lift_digging_end_position", -3150)  # Measured in encoder counts
        self.declare_parameter("skimmer_belt_power", -0.3)

        self.lift_dumping_position = self.get_parameter("lift_dumping_position").value * 360 / 42
        self.lift_digging_start_position = self.get_parameter("lift_digging_start_position").value * 360 / 42
        self.lift_digging_end_position = self.get_parameter("lift_digging_end_position").value * 360 / 42
        self.skimmer_belt_power = self.get_parameter("skimmer_belt_power").value

        self.cli_drivetrain_drive = self.create_client(Drive, "drivetrain/drive")
        self.cli_drivetrain_stop = self.create_client(Stop, "drivetrain/stop")
        self.drivetrain_future = Future()

        self.cli_lift_zero = self.create_client(Stop, "lift/zero")
        self.cli_lift_setPosition = self.create_client(SetPosition, "lift/setPosition")
        self.cli_lift_set_power = self.create_client(SetPower, "lift/setPower")
        self.cli_lift_stop = self.create_client(Stop, "lift/stop")
        self.lift_future = Future()

        self.cli_skimmer_stop = self.create_client(Stop, "skimmer/stop")
        self.cli_skimmer_setPower = self.create_client(SetPower, "skimmer/setPower")
        self.skimmer_future = Future()

        self.canceled = False

    def execute_callback(self, goal_handle: AutoDig.Goal):
        self.get_logger().info("Starting Autonomous Digging Procedure!")
        result = AutoDig.Result()

        # Zero the skimmer
        self.lift_future = self.cli_lift_zero.call_async(Stop.Request())
        rclpy.spin_until_future_complete(self, self.lift_future)
        if self.lift_future.cancelled():
            self.get_logger().error("Failed to Zero Lift")
            self.canceled = False
            return result

        # Lower the skimmer onto the ground=
        if not self.cli_lift_setPosition.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Failed to get lift set positon service")
            return result
        self.lift_future = self.cli_lift_setPosition.call_async(
            SetPosition.Request(position=self.lift_digging_start_position)
        )
        rclpy.spin_until_future_complete(self, self.lift_future)
        if self.lift_future.cancelled():
            self.get_logger().error("Failed to Lower Lift")
            self.canceled = False
            return result

        # Start the skimmer belt
        self.skimmer_future = self.cli_skimmer_setPower.call_async(SetPower.Request(power=self.skimmer_belt_power))

        # Drive forward while digging
        start_time = self.get_clock().now().nanoseconds
        elapsed = self.get_clock().now().nanoseconds - start_time
        # accelerate for 2 seconds
        while elapsed < 2e9 and not self.canceled:
            self.lift_future = self.cli_lift_set_power.call_async(SetPower.Request(power=-0.05e-9 * (elapsed)))
            self.drivetrain_future = self.cli_drivetrain_drive.call_async(
                Drive.Request(forward_power=0.0, horizontal_power=0.25e-9 * (elapsed), turning_power=0.0)
            )
            self.get_logger().info("Accelerating lift and drive train")
            elapsed = self.get_clock().now().nanoseconds - start_time
            time.sleep(0.1)  # TODO make the ramp up a service so this while loop isn't needed

        if self.canceled:
            self.get_logger().info("Digging Canceled")
            self.canceled = False
            return result

        self.get_logger().info("Auto Driving")

        # keep digging at full speed for the remaining 10 seconds
        while self.get_clock().now().nanoseconds - start_time < 12e9 and not self.canceled:
            # Allows other the task to be cancelled there is probably a better way to do this
            time.sleep(0.1)

        if self.canceled:
            self.get_logger().info("Digging Canceled")
            self.canceled = False
            return result

        # Stop driving and skimming
        self.drivetrain_future = self.cli_drivetrain_stop.call_async(Stop.Request())
        self.skimmer_future = self.cli_skimmer_stop.call_async(Stop.Request())

        self.lift_future = self.cli_lift_setPosition.call_async(
            SetPosition.Request(position=self.lift_dumping_position)
        )
        rclpy.spin_until_future_complete(self, self.lift_future)
        if self.lift_future.cancelled():
            self.get_logger().error("Failed to Raise Lift")
            return result

        self.get_logger().info("Autonomous Digging Procedure Complete!\n")

        return result

    def cancel_callback(self, goal_handle):
        """This method is called when the action is cancelled."""
        self.canceled = True
        self.lift_future.cancel()
        self.skimmer_future.cancel()
        self.drivetrain_future.cancel()
        self.cli_skimmer_stop.call_async(Stop.Request())
        self.cli_drivetrain_stop.call_async(Stop.Request())
        self.cli_lift_stop.call_async(Stop.Request())
        self.get_logger().info(f"Goal {type(goal_handle)} was cancelled!")
