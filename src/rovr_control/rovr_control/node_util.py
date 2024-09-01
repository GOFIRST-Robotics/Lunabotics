from rclpy.action.server import CancelResponse, ServerGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from std_msgs.msg import Bool


class AsyncNode(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.skimmer_goal_reached = Future()
        self.skimmer_goal_reached.set_result(None)

        self.sleep_goal_reached = Future()
        self.sleep_goal_reached.set_result(None)

        self.timer = None

    # TODO: This should not be needed anymore after ticket #257 is implemented!
    def skimmer_goal_callback(self, msg: Bool) -> None:
        """Update the member variable accordingly."""
        if msg.data:
            self.skimmer_goal_reached.set_result(None)

    # TODO: This should not be needed anymore after ticket #257 is implemented!
    async def skimmer_sleep(self) -> None:
        self.skimmer_goal_reached = Future()
        await self.skimmer_goal_reached

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
