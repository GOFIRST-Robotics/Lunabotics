import rclpy
from rclpy.action import ActionServer
from rclpy.client import Future
from rclpy.node import Node

from rclpy.action.server import CancelResponse

from rovr_interfaces.action import AutoOffload
from rovr_interfaces.srv import Drive, Stop, SetPosition, SetPower

import time


class AutoOffloadServer(Node):
    def __init__(self):
        super().__init__("auto_offload_server")
        self._action_server = ActionServer(
            self, AutoOffload, "auto_offload", self.execute_callback, cancel_callback=self.cancel_callback
        )
        self.client_node = rclpy.create_node("auto_offload_action_server_client")

        self.cli_drivetrain_drive = self.client_node.create_client(Drive, "drivetrain/drive")
        self.cli_drivetrain_stop = self.client_node.create_client(Stop, "drivetrain/stop")
        self.future_drivetrain = Future()

        self.cli_lift_setPosition = self.client_node.create_client(SetPosition, "lift/set_position")
        self.cli_lift_stop = self.client_node.create_client(Stop, "lift/stop")
        self.future_lift = Future()

        self.cli_skimmer_setPower = self.client_node.create_client(SetPower, "skimmer/set_power")
        self.cli_skimmer_stop = self.client_node.create_client(Stop, "skimmer/stop")
        self.future_skimmer = Future()

        self.canceled = False

    def execute_callback(self, goal_handle: AutoOffload.Goal):
        """This method lays out the procedure for autonomously offloading!"""
        result = AutoOffload.Result()

        # Drive backward into the berm zone
        self.future_drivetrain = self.cli_drivetrain_drive.call_async(
            Drive.Request(forward_power=0.0, horizontal_power=-0.25, turning_power=0.0)
        )
        time_to_drive = 10e9
        start_time = self.get_clock().now().nanoseconds
        self.get_logger().info("Auto Driving")
        while self.get_clock().now().nanoseconds - start_time < time_to_drive:
            # Allows for task to be cancelled
            time.sleep(0.1)
            if self.canceled:
                self.get_logger().error("Driving Procedure Cancelled!\n")
                self.canceled = False
                goal_handle.canceled()
                return result
        self.future_drivetrain = self.cli_drivetrain_stop.call_async(Stop.Request())

        # Raise up the skimmer in preparation for dumping
        self.future_lift = self.cli_lift_setPosition.call_async(
            SetPosition.Request(position=goal_handle.request.lift_dumping_position)
        )
        # Wait for the lift goal to be reached
        rclpy.spin_until_future_complete(self.client_node, self.future_lift)
        if self.future_lift.cancelled():
            self.get_logger().error("Raise Skimmer Cancelled!\n")
            self.canceled = False
            goal_handle.canceled()
            return result

        self.future_skimmer = self.cli_skimmer_setPower.call_async(
            SetPower.Request(power=goal_handle.request.skimmer_belt_power)
        )
        time_to_offload = int(
            1.0 / abs(goal_handle.request.skimmer_belt_power) * 1e9
        )  # convert from seconds to nanoseconds
        start_time = self.get_clock().now().nanoseconds
        self.get_logger().info("Commence Offloading!")
        while self.get_clock().now().nanoseconds - start_time < time_to_offload:
            # Allows for task to be cancelled
            time.sleep(0.1)
            if self.canceled:
                self.get_logger().error("Offload Belt Cancelled!\n")
                self.canceled = False
                goal_handle.canceled()
                return result

        self.future_skimmer = self.cli_skimmer_stop.call_async(Stop.Request())  # Stop the skimmer belt

        self.get_logger().info("Autonomous Offload Procedure Complete!\n")
        goal_handle.succeed()
        return result

    def cancel_callback(self, cancel_request):
        """This method is called when the action is cancelled."""
        self.canceled = True
        self.get_logger().info("Goal was canceled")
        self.future_drivetrain.cancel()
        self.future_lift.cancel()
        self.future_skimmer.cancel()
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    action_server = AutoOffloadServer()

    rclpy.spin(action_server)


if __name__ == "__main__":
    main()
