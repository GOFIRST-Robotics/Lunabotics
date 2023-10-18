# This ROS 2 node contains code for the conveyor subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2023

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet
from rovr_interfaces.srv import SetPower, Stop, SetHeight


class ConveyorNode(Node):
    def __init__(self):
        """Initialize the ROS 2 conveyor node."""
        super().__init__("conveyor")

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")

        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(SetPower, "conveyor/toggle", self.toggle_callback)
        self.srv_stop = self.create_service(Stop, "conveyor/stop", self.stop_callback)
        self.srv_setPower = self.create_service(SetPower, "conveyor/setPower", self.set_power_callback)
        self.srv_setHeight = self.create_service(SetHeight, "conveyor/setHeight", self.set_height_callback)
        
        # TODO: Create a publisher that publishes the current conveyor height as a float32 to /conveyor/height
        # TODO: Create another publisher that publishes a bool to /conveyor/goal_reached describing if the latest height goal has been reached
        
        # TODO: create a timer that runs every 100ms and publishes messages using the publishers defined above

        # Define motor CAN IDs here
        self.CONVEYOR_BELT_MOTOR = 6
        self.HEIGHT_ADJUST_MOTOR = 420

        # Current state of the conveyor belt
        self.running = False
        # Current goal height
        self.current_goal_height = 0 # relative encoders always initialize to 0
        # Goal Threshold (if abs(self.current_goal_height - ACTUAL VALUE) <= self.goal_threshold then we should publish True to /conveyor/goal_reached)
        self.goal_threshold = 50

    # Define subsystem methods here
    def set_power(self, conveyor_power: float) -> None:
        """This method sets power to the conveyor belt."""
        self.running = True
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.CONVEYOR_BELT_MOTOR, value=conveyor_power)
        )

    def stop(self) -> None:
        """This method stops the conveyor belt."""
        self.running = False
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.CONVEYOR_BELT_MOTOR, value=0.0)
        )

    def toggle(self, conveyor_belt_power: float) -> None:
        """This method toggles the conveyor belt."""
        if self.running:
            self.stop()
        else:
            self.set_power(conveyor_belt_power)

    def set_height(self, height: float) -> None:
        """This method sets the height of the conveyor."""
        self.current_goal_height = height
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="position", can_id=self.HEIGHT_ADJUST_MOTOR, value=height)
        )
        
    # TODO: Create a method to stop the height adjust pulley
    
    # TODO: Create a method to set power (duty cycle) to the height adjust pulley for manual control

    # Define service callback methods here
    def set_power_callback(self, request, response):
        """This service request sets power to the conveyor belt."""
        self.set_power(request.conveyor_belt_power)
        response.success = 0  # indicates success
        return response

    def stop_callback(self, request, response):
        """This service request stops the conveyor belt."""
        self.stop()
        response.success = 0  # indicates success
        return response

    def toggle_callback(self, request, response):
        """This service request toggles the conveyor belt."""
        self.toggle(request.conveyor_belt_power)
        response.success = 0  # indicates success
        return response

    def set_height_callback(self, request, response):
        """This service request sets the height of the conveyor."""
        self.set_height(request.height)
        response.success = 0  # indicates success
        return response
    
    # TODO: Create a service wrapper for the method to stop the height adjust pulley
    
     # TODO: Create a service wrapper for the method to set power (duty cycle) to the height adjust pulley

    # Define timer callback methods here
    # TODO: Create a timer callback method for the 100ms timer that publishes using the 2 publishers


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = ConveyorNode()
    node.get_logger().info("Initializing the Conveyor subsystem!")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
