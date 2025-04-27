import time
import unittest
import uuid

import rclpy
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from unique_identifier_msgs.msg import UUID
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus
from rovr_interfaces.action import AutoDig
from rovr_interfaces.srv import SetPosition, SetPower
from rclpy.action.server import ServerGoalHandle, CancelResponse
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rovr_control.auto_dig_server import AutoDigServer

class TestAutoDig(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = MultiThreadedExecutor(context=cls.context)
        cls.node = AutoDigServer(context=cls.context)
        # cls.cg = MutuallyExclusiveCallbackGroup()
        cls.ac = ActionClient(cls.node, AutoDig, "auto_dig")
        cls.goal = AutoDig.Goal(
            lift_digging_start_position=0.0,
            lift_digging_end_position=1.0,
            digger_chain_power=0.0,
        )

        def simple_callback(request, response):
            response.success = True
            return response
        cls.cli_lift_zero = cls.node.create_service(Trigger, "lift/zero", simple_callback)
        cls.cli_lift_bottom = cls.node.create_service(Trigger, "lift/bottom", simple_callback)
        cls.cli_lift_setPosition = cls.node.create_service(
            SetPosition, "lift/setPosition", simple_callback
        )
        cls.cli_lift_set_power = cls.node.create_service(
            SetPower, "lift/setPower", simple_callback
        )
        cls.cli_lift_stop = cls.node.create_service(Trigger, "lift/stop", simple_callback)

        cls.cli_digger_stop = cls.node.create_service(Trigger, "digger/stop", simple_callback)
        cls.cli_digger_setPower = cls.node.create_service(
            SetPower, "digger/setPower", simple_callback
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def setUp(self) -> None:
        self.feedback = None

    def timed_spin(self, duration):
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

    def test_autodig(self) -> None:
        # Defaults
        auto_dig_handle = self.ac.send_goal_async(self.goal)
        rclpy.spin_until_future_complete(self.node, auto_dig_handle, self.executor, timeout_sec=1)
        result_handle: ClientGoalHandle = auto_dig_handle.result()
        result_future: Future = result_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, self.executor, timeout_sec=10)
        self.assertTrue(result_handle.status == GoalStatus.STATUS_SUCCEEDED)

    def test_autodig_cancel(self) -> None:
        # Defaults
        auto_dig_handle = self.ac.send_goal_async(self.goal)
        rclpy.spin_until_future_complete(self.node, auto_dig_handle, self.executor, timeout_sec=1)
        result_handle: ClientGoalHandle = auto_dig_handle.result()
        result_future: Future = result_handle.get_result_async()
        result_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, result_future, self.executor, timeout_sec=10)
        self.assertTrue(result_handle.status == GoalStatus.STATUS_CANCELED)


if __name__ == "__main__":
    unittest.main()
