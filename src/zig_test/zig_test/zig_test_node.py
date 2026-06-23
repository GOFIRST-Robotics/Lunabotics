# This ROS 2 node contains the code for the zig test subsystem of the robot
import time
import math

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import (
    Addition
)


class ZigTest(Node):
    def __init__(self):
        "Initialize the ROS 2 Zig Test node"
        super().__init__("zig_test")

        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        self.srv_add = self.create_service(
            Addition,
            "zig_test/addition",
            self.addition,
            callback_group=self.service_cb_group,
        )
    
    def addition(self, request, response):
        response.sum = request.a + request.b
        print(f"{request.a} + {request.b} = {response.sum}")
        return response



def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = ZigTest()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    node.get_logger().info("Initializing the Zig Test subsystem!")
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down the Zig Test subsystem.")
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

