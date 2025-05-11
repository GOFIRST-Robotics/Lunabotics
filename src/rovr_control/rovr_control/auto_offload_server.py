import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, ServerGoalHandle

from rovr_interfaces.action import AutoOffload
from std_srvs.srv import Trigger, SetBool

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

        self.cli_dumper_extend = self.create_client(Trigger, "dumper/extendDumper")
        self.cli_dumper_retract = self.create_client(Trigger, "dumper/retractDumper")
        self.cli_dumper_stop = self.create_client(Trigger, "dumper/stop")
        self.cli_motor_toggle = self.create_client(Trigger, "motor_toggle")

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """This method lays out the procedure for autonomously offloading!"""
        self.get_logger().info("Starting Autonomous Offload Procedure!")
        result = AutoOffload.Result()

        # Make sure the services are available
        if not self.cli_dumper_extend.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Dumper extend service not available")
            goal_handle.abort()
            return result
        if not self.cli_dumper_retract.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Dumper retract service not available")
            goal_handle.abort()
            return result
        if not self.cli_dumper_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Dumper stop service not available")
            goal_handle.abort()
            return result
        if not self.cli_motor_toggle.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Agitator motor toggle service not available")
            goal_handle.abort()
            return result

        # Dump the material
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Auto Dumping")
            await self.cli_dumper_extend.call_async(Trigger.Request())
        if not goal_handle.is_cancel_requested:
            # wait for 5 seconds before retracting the dumper
            await self.async_sleep(5)  # Allows for task to be canceled
        if not goal_handle.is_cancel_requested:
            # retract the dumper
            await self.cli_dumper_retract.call_async(Trigger.Request())

        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Autonomous Offload Procedure Complete!")
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
        self.cli_dumper_stop.call_async(Trigger.Request())
        return CancelResponse.ACCEPT


def main(args=None) -> None:
    rclpy.init(args=args)

    action_server = AutoOffloadServer()
    rclpy.spin(action_server)

    action_server.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
