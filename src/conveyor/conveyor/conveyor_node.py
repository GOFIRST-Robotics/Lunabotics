# This ROS 2 node contains code for the conveyor subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2023

# Import the ROS 2 Python module
from std_msgs.msg import Float32, Bool
import rclpy
from rclpy.node import Node

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet, MotorCommandGet
from rovr_interfaces.srv import SetPower, Stop, SetHeight

# from std_msgs.msg import Bool, Float32


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
        self.srv_stop_height_adjust = self.create_service(Stop, "pulley/stop", self.stop_height_callback)
        self.srv_set_power_Pulley = self.create_service(SetPower, "pulley/setPower", self.set_power_pulley_callback)
        self.publisher_height = self.create_publisher(Float32, "/conveyor/height")
        self.publisher_goal_reached = self.create_publisher(Bool, "/conveyor/goal_reached")

        # Define timers here
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Define motor CAN IDs here
        self.CONVEYOR_BELT_MOTOR = 9
        self.HEIGHT_ADJUST_MOTOR = 10

        # Current state of the conveyor belt
        self.running = False
        # Current goal height
        self.current_goal_height = 0  # relative encoders always initialize to 0
        # Goal Threshold (if abs(self.current_goal_height - ACTUAL VALUE) <= self.goal_threshold then we should publish True to /conveyor/goal_reached)
        self.goal_threshold = 0.1
        # Current state of the pulley system
        self.pulley_running = False
        # ----------------------------------------------------------------
        # Circumference of height adjust motor
        self.height_adjust_circumference = (
            0.1  # meters # NOT FIXED: can be changed based on whatever the mechanical ppl want
        )
        # Gear ratio
        self.gear_ratio = 1 / 1  # NOT FIXED: can be changed based on whatever the mechanical ppl want
        # motor rotate 360 degrees -> self.height_adjust_circumference / self.gear_ratio
        # ----------------------------------------------------------------

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
        height_degrees = (self.gear_ratio / height) * 360
        self.current_goal_height = height  # goal height should be in meters
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="position", can_id=self.HEIGHT_ADJUST_MOTOR, value=height_degrees)
        )

    def stop_height_adjust(self) -> None:
        """This method stops the pulley."""
        self.pulley_running = False
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.HEIGHT_ADJUST_MOTOR, value=0.0)
        )

    def set_power_Pulley(self, power: float) -> None:
        """This method sets power to the pulley."""
        self.pulley_running = True
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.HEIGHT_ADJUST_MOTOR, value=power)
        )

    # Define service callback methods here
    def set_power_callback(self, request, response):
        """This service request sets power to the conveyor belt."""
        self.set_power(request.power)
        response.success = 0  # indicates success
        return response

    def stop_callback(self, request, response):
        """This service request stops the conveyor belt."""
        self.stop()
        response.success = 0  # indicates success
        return response

    def toggle_callback(self, request, response):
        """This service request toggles the conveyor belt."""
        self.toggle(request.power)
        response.success = 0  # indicates success
        return response

    def set_height_callback(self, request, response):
        """This service request sets the height of the conveyor."""
        self.set_height(request.height)
        response.success = 0  # indicates success
        return response

    def stop_height_callback(self, request, response):
        """This service request stops the pulley."""
        self.stop_height_adjust()
        response.success = 0  # indicates success
        return response

    def set_power_pulley_callback(self, request, response):
        """This service request sets power to the conveyor belt."""
        self.set_power_Pulley(request.power)
        response.success = 0  # indicates success
        return response

    # Define timer callback methods here
    def timer_callback(self):
        """Publishes the current height"""
        # The value returned by the MotorCommandGet service will be in degrees
        height = MotorCommandGet.Request(type="position", can_id=self.HEIGHT_ADJUST_MOTOR)
        height_meters = height / (360 * self.gear_ratio)
        msg_height = Float32()
        msg_height.data = height_meters
        self.publisher_height.publish(msg_height)

        msg_goal_reached = Bool()
        msg_goal_reached.data = abs(self.current_goal_height - height_meters) <= self.goal_threshold
        self.publisher_goal_reached.publish(msg_goal_reached)


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
