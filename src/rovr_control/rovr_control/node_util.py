from rclpy.action.server import CancelResponse, ServerGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from std_msgs.msg import Bool


class AsyncNode(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.sleep_goal_reached = Future()
        self.sleep_goal_reached.set_result(None)

        self.timer = None

    # Temporary measure to sleep for time without needlessly spinning
    async def async_sleep(self, seconds: float) -> None:
        self.sleep_goal_reached = Future()

        def handler():
            self.sleep_goal_reached.set_result(None)

        self.timer = self.create_timer(seconds, handler)
        await self.sleep_goal_reached
        self.timer.cancel()
        self.timer.destroy()

    def cancel_callback(self, cancel_request: ServerGoalHandle) -> None:
        if not self.sleep_goal_reached.done():
            self.timer.cancel()
            self.timer.destroy()
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT