import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, ServerGoalHandle

from rovr_interfaces.action import AutoOffload
from rovr_interfaces.srv import Stop


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

        self.cli_dumper_stop = self.create_client(Trigger, "dumper/stop")
        #self.cli_dumper_dump = self.create_client(Trigger, "dumper/dump")
        self.cli_dumper_extend = self.create_client(Trigger, "dumper/extendDumper")
        self.cli_dumper_retract = self.create_client(Trigger, "dumper/retractDumper")

        self.declare_parameter("dump_time", 5)
        self.dumpTime = self.get_parameter("dump_time").value


    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """This method lays out the procedure for autonomously offloading!"""
        self.get_logger().info("Starting Autonomous Offload Procedure!")
        result = AutoOffload.Result()

        # Make sure the services are available
        if not self.cli_dumper_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Dumper stop service not available")
            goal_handle.abort()
            return result
        if not self.cli_dumper_dump.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Dumper dump service not available")
            goal_handle.abort()
            return result

        await self.cli_dumper_extend.call_async(Trigger.Request())  # Dump Berm

        await self.async_sleep(self.dumpTime)

        await self.cli_dumper_retract.call_async(Trigger.Request())  # Stop the dumper system
        self.get_logger().info("Autonomous Offload Procedure Complete!")
        goal_handle.succeed()
        return result

    def cancel_callback(self, cancel_request: ServerGoalHandle):
        """This method is called when the action is canceled."""
        self.get_logger().info("Goal is cancelling")
        # If lift is raising stop it
        self.cli_dumper_stop.call_async(Stop.Request())
        return CancelResponse.ACCEPT


def main(args=None) -> None:
    rclpy.init(args=args)
    action_server = AutoOffloadServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
