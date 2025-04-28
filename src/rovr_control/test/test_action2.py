import time
import unittest
import threading

import rclpy
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from rovr_interfaces.action import GoToDigLocation
from rovr_interfaces.srv import SetPosition, SetPower
from nav2_msgs.srv import GetCostmap
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
from rclpy.action.client import ClientGoalHandle
from rovr_control.dig_location_server import DigLocationFinder
from rclpy.parameter import Parameter

from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from nav2_msgs.action import NavigateToPose


class TestGoToDig(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        # Initialize ROS
        rclpy.init(context=cls.context)

        # Create node and executor
        cls.node = DigLocationFinder(context=cls.context)
        cls.executor = MultiThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)

        # # Set up sim time
        # cls.node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        # cls.clock_pub = cls.node.create_publisher(Clock, "/clock", 10)

        # Setup mock services that will force action completion
        def simple_callback(request, response):
            if hasattr(response, "success"):
                response.success = True
            return response

        # Create all necessary mock services
        cls.get_costmap_global_srv = cls.node.create_service(
            GetCostmap, "global_costmap/get_costmap", simple_callback
        )

        def navigate_execute_callback(goal_handle):
            cls.node.get_logger().info("Executing navigation goal")
            # Store the goal handle for later checking

            # Allow some time to process cancel requests
            time.sleep(0.2)

            # If not canceled, complete with success
            if not goal_handle.is_cancel_requested:
                result = NavigateToPose.Result()
                goal_handle.succeed()
                return result
            else:
                # If canceled, return empty result
                return NavigateToPose.Result()

        cls.nav2_server = ActionServer(
            cls.node,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=navigate_execute_callback,
        )

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

        # Create action client
        cls.ac = ActionClient(cls.node, GoToDigLocation, "go_to_dig_location")

        # Ensure the action server is available - this is critical
        cls.node.get_logger().info("Waiting for action server...")
        server_ready = cls.ac.wait_for_server(timeout_sec=3.0)
        if not server_ready:
            cls.node.get_logger().warning("Action server not available after timeout!")
            raise RuntimeError("Action server not available")

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.context)

    def test_go_to_dig(self):
        """Test the successful completion of a go_to_dig_location action."""
        goal = GoToDigLocation.Goal()

        request_handle = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, request_handle, self.executor, timeout_sec=1)
        result_handle: ClientGoalHandle = request_handle.result()
        result_future: Future = result_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, self.executor, timeout_sec=10)
        self.assertTrue(result_handle.status == GoalStatus.STATUS_SUCCEEDED)

    def test_go_to_dig_cancel(self):
        """Test cancellation of a go_to_dig_location action."""
        # Create a goal
        goal = GoToDigLocation.Goal()
        request_handle = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, request_handle, self.executor, timeout_sec=1)
        result_handle: ClientGoalHandle = request_handle.result()
        result_future: Future = result_handle.get_result_async()
        result_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, result_future, self.executor, timeout_sec=10)
        self.assertTrue(result_handle.status == GoalStatus.STATUS_CANCELED)


if __name__ == "__main__":
    unittest.main()
