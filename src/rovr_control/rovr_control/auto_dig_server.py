import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rclpy.action.server import CancelResponse
from std_msgs.msg import Bool

from rovr_interfaces.action import AutoDig
from rovr_interfaces.srv import Drive, Stop, SetPosition, SetPower

import time


class AutoDigServer(Node):
    def __init__(self):
        super().__init__("auto_dig_server")
        self._action_server = ActionServer(
            self, AutoDig, "auto_dig", self.execute_callback, cancel_callback=self.cancel_callback
        )
        self.client_node = rclpy.create_node("auto_offload_action_server_client")

        # TODO: This should not be needed anymore after ticket #257 is implemented!
        self.skimmer_goal_subscription = self.client_node.create_subscription(
            Bool, "/skimmer/goal_reached", self.skimmer_goal_callback, 10
        )

        self.cli_drivetrain_drive = self.client_node.create_client(Drive, "drivetrain/drive")
        self.cli_drivetrain_stop = self.client_node.create_client(Stop, "drivetrain/stop")

        self.cli_lift_zero = self.client_node.create_client(Stop, "lift/zero")
        self.cli_lift_setPosition = self.client_node.create_client(SetPosition, "lift/setPosition")
        self.cli_lift_set_power = self.client_node.create_client(SetPower, "lift/setPower")
        self.cli_lift_stop = self.client_node.create_client(Stop, "lift/stop")

        self.cli_skimmer_stop = self.client_node.create_client(Stop, "skimmer/stop")
        self.cli_skimmer_setPower = self.client_node.create_client(SetPower, "skimmer/setPower")

        self.canceled = False

    def execute_callback(self, goal_handle: AutoDig.Goal):
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
        if not self.cli_skimmer_setPower.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Skimmer set power service not available")
            goal_handle.abort()
            return result
        if not self.cli_skimmer_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Skimmer stop service not available")
            goal_handle.abort()
            return result

        # Zero the skimmer
        self.cli_lift_zero.call_async(Stop.Request())
        # Wait for the lift goal to be reached
        self.skimmer_goal_reached = False
        while not self.skimmer_goal_reached:
            self.get_logger().info("Moving the lift to the goal")
            rclpy.spin_once(self.client_node, timeout_sec=0)  # Allows for task to be canceled
            if self.canceled:
                self.get_logger().warn("Zeroing the Lift was Canceled!\n")
                self.canceled = False
                goal_handle.canceled()
                return result
            time.sleep(0.1)

        # Lower the skimmer onto the ground
        self.cli_lift_setPosition.call_async(
            SetPosition.Request(position=goal_handle.request.lift_digging_start_position)
        )
        # Wait for the lift goal to be reached
        self.skimmer_goal_reached = False
        while not self.skimmer_goal_reached:
            self.get_logger().info("Moving the lift to the goal")
            rclpy.spin_once(self.client_node, timeout_sec=0)  # Allows for task to be canceled
            if self.canceled:
                self.get_logger().warn("Lowering the Lift was Canceled!\n")
                self.canceled = False
                goal_handle.canceled()
                return result
            time.sleep(0.1)

        # Start the skimmer belt
        self.cli_skimmer_setPower.call_async(SetPower.Request(power=goal_handle.request.skimmer_belt_power))

        # Drive forward while digging
        start_time = self.get_clock().now().nanoseconds
        elapsed = self.get_clock().now().nanoseconds - start_time
        # accelerate for 2 seconds
        # TODO make the ramp up a service so this while loop isn't needed
        while elapsed < 2e9:
            self.cli_lift_set_power.call_async(SetPower.Request(power=-0.05e-9 * (elapsed)))
            self.cli_drivetrain_drive.call_async(
                Drive.Request(forward_power=0.0, horizontal_power=0.25e-9 * (elapsed), turning_power=0.0)
            )
            self.get_logger().info("Accelerating lift and drive train")
            elapsed = self.get_clock().now().nanoseconds - start_time
            rclpy.spin_once(self.client_node, timeout_sec=0)  # Allows for task to be canceled
            if self.canceled:
                self.get_logger().warn("Digging Acceleration Canceled!\n")
                self.canceled = False
                goal_handle.canceled()
                return result
            time.sleep(0.1)

        self.get_logger().info("Auto Driving")
        # keep digging at full speed for the remaining 10 seconds
        while self.get_clock().now().nanoseconds - start_time < 12e9:
            rclpy.spin_once(self.client_node, timeout_sec=0)  # Allows for task to be canceled
            if self.canceled:
                self.get_logger().warn("Digging Canceled!\n")
                self.canceled = False
                goal_handle.canceled()
                return result
            time.sleep(0.1)

        # Stop driving and skimming
        self.cli_drivetrain_stop.call_async(Stop.Request())
        self.cli_skimmer_stop.call_async(Stop.Request())

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

        self.get_logger().info("Autonomous Digging Procedure Complete!\n")
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
    action_server = AutoDigServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
