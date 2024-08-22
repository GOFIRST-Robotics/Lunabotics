import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rclpy.action.server import CancelResponse
from std_msgs.msg import Bool

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

        # TODO: This should not be needed anymore after ticket #257 is implemented!
        self.skimmer_goal_subscription = self.client_node.create_subscription(
            Bool, "/skimmer/goal_reached", self.skimmer_goal_callback, 10
        )

        self.cli_drivetrain_drive = self.client_node.create_client(Drive, "drivetrain/drive")
        self.cli_drivetrain_stop = self.client_node.create_client(Stop, "drivetrain/stop")

        self.cli_lift_setPosition = self.client_node.create_client(SetPosition, "lift/setPosition")
        self.cli_lift_stop = self.client_node.create_client(Stop, "lift/stop")

        self.cli_skimmer_setPower = self.client_node.create_client(SetPower, "skimmer/setPower")
        self.cli_skimmer_stop = self.client_node.create_client(Stop, "skimmer/stop")

        self.canceled = False

    def execute_callback(self, goal_handle: AutoOffload.Goal):
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
        self.cli_drivetrain_drive.call_async(
            Drive.Request(forward_power=0.0, horizontal_power=-0.25, turning_power=0.0)
        )
        time_to_drive = 10e9
        start_time = self.get_clock().now().nanoseconds
        self.get_logger().info("Auto Driving")
        while self.get_clock().now().nanoseconds - start_time < time_to_drive:
            rclpy.spin_once(self.client_node, timeout_sec=0)  # Allows for task to be canceled
            if self.canceled:
                self.get_logger().warn("Driving Procedure Canceled!\n")
                self.canceled = False
                goal_handle.canceled()
                return result
            time.sleep(0.1)
        self.cli_drivetrain_stop.call_async(Stop.Request())

        # Raise up the skimmer in preparation for dumping
        self.get_logger().info("Raising the Lift!")
        self.cli_lift_setPosition.call_async(SetPosition.Request(position=goal_handle.request.lift_dumping_position))
        # Wait for the lift goal to be reached
        self.skimmer_goal_reached = False
        while not self.skimmer_goal_reached:
            self.get_logger().info("Moving the lift to the goal")
            rclpy.spin_once(self.client_node, timeout_sec=0)  # Allows for task to be canceled
            if self.canceled:
                self.get_logger().warn("Raising the Lift was Canceled!\n")
                self.canceled = False
                goal_handle.canceled()
                return result
            time.sleep(0.1)

        self.cli_skimmer_setPower.call_async(SetPower.Request(power=goal_handle.request.skimmer_belt_power))
        time_to_offload = 1.0 / abs(goal_handle.request.skimmer_belt_power) * 1e9  # convert from seconds to nanoseconds
        start_time = self.get_clock().now().nanoseconds
        self.get_logger().info("Commence Offloading!")
        while self.get_clock().now().nanoseconds - start_time < time_to_offload:
            rclpy.spin_once(self.client_node, timeout_sec=0)  # Allows for task to be canceled
            if self.canceled:
                self.get_logger().warn("Offload Belt was Canceled!\n")
                self.canceled = False
                goal_handle.canceled()
                return result
            time.sleep(0.1)

        self.cli_skimmer_stop.call_async(Stop.Request())  # Stop the skimmer belt

        self.get_logger().info("Autonomous Offload Procedure Complete!\n")
        goal_handle.succeed()
        return result

    def cancel_callback(self, cancel_request):
        """This method is called when the action is canceled."""
        self.get_logger().info("Goal was canceled")
        self.canceled = True
        return CancelResponse.ACCEPT

    # TODO: This should not be needed anymore after ticket #257 is implemented!
    def skimmer_goal_callback(self, msg: Bool) -> None:
        """Update the member variable accordingly."""
        self.skimmer_goal_reached = msg.data


def main(args=None) -> None:
    rclpy.init(args=args)
    action_server = AutoOffloadServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
