import rclpy
from rclpy.action import ActionServer
from rclpy.client import Future
from rclpy.node import Node

from rovr_interfaces.action import AutoOffload
from rovr_interfaces.srv import Drive, Stop, SetPosition, SetPower

import time

class AutoOffloadServer(Node):
    def __init__(self):
        super().__init__("auto_offload_server")
        self._action_server = ActionServer(
            self, AutoOffload, "auto_offload", self.execute_callback, cancel_callback=self.cancel_callback
        )

        self.cli_drivetrain_drive = self.create_client(Drive, "drivetrain/drive")
        self.cli_drivetrain_stop = self.create_client(Stop, "drivetrain/stop")
        self.future_drivetrain = Future()

        self.cli_lift_setPosition = self.create_client(SetPosition, "lift/set_position")
        self.cli_lift_stop = self.create_client(Stop, "lift/stop")
        self.future_lift = Future()

        self.cli_skimmer_setPower = self.create_client(SetPower, "skimmer/set_power")
        self.cli_skimmer_stop = self.create_client(Stop, "skimmer/stop")
        self.future_skimmer = Future()

        self.canceled = False

    def execute_callback(self, goal_handle: AutoOffload.Goal):
        """This method lays out the procedure for autonomously offloading!"""
        result = AutoOffload.Result()

        # Drive backward into the berm zone
        self.future_drivetrain = self.cli_drivetrain_drive.call_async(Drive.Request(forward_power=0.0, horizontal_power=-0.25, turning_power=0.0))
        start_time = self.get_clock().now().nanoseconds
        self.get_logger().info("Auto Driving")
        while self.get_clock().now().nanoseconds - start_time < 10e9 and not self.canceled:
            # Allows for task to be cancelled
            time.sleep(0.1)
        if self.canceled:
            self.get_logger().error("Driving Procedure Cancelled!\n")
            self.canceled = False
            return result
        self.future_drivetrain = self.cli_drivetrain_stop.call_async(Stop.Request())

        # Raise up the skimmer in preparation for dumping
        self.future_lift = self.cli_lift_setPosition.call_async(SetPosition.Request(position=goal_handle.lift_dumping_position))
        # Wait for the lift goal to be reached
        rclpy.spin_until_future_complete(self, self.future_lift)
        if self.future_lift.cancelled():
            self.get_logger().error("Raise Skimmer Cancelled!\n")
            self.canceled = False
            return result
        
        self.get_logger().info("Commence Offloading!")
        self.future_skimmer = self.cli_skimmer_setPower.call_async(SetPower.Request(power=goal_handle.skimmer_belt_power))
        count = 0
        time_to_offload = 8
        while not self.canceled and count < time_to_offload:
            # Allows for task to be cancelled
            time.sleep(1.0/goal_handle.skimmer_belt_power)  # How long to offload for
            count += 1
        if self.canceled:
            self.get_logger().error("Offload Belt Cancelled!\n")
            self.canceled = False
            return result
    
        self.future_skimmer = self.cli_skimmer_stop.call_async(Stop.Request())  # Stop the skimmer belt
        self.get_logger().info("Autonomous Offload Procedure Complete!\n")

        return result

    def cancel_callback(self, goal_handle):
        """This method is called when the action is cancelled."""
        self.canceled = True
        self.future_drivetrain.cancel()
        self.future_lift.cancel()
        self.future_skimmer.cancel()
        self.future_drivetrain = self.cli_drivetrain_stop.call_async(Stop.Request())
        self.future_lift = self.cli_lift_stop.call_async(Stop.Request())
        self.future_skimmer = self.cli_skimmer_stop.call_async(Stop.Request())
        self.get_logger().info(f"Goal {type(goal_handle)} was cancelled!")


def main(args=None):
    rclpy.init(args=args)

    action_server = AutoOffloadServer()

    rclpy.spin(action_server)


if __name__ == "__main__":
    main()
