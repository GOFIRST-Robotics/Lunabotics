# This ROS 2 node contains code for the digger subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Charlie Parece <parec020@umn.edu>
# Last Updated: October 2024

# Import the ROS 2 Python module
from warnings import deprecated
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet, MotorCommandGet
from rovr_interfaces.srv import SetPower, SetPosition
from rovr_interfaces.msg import LimitSwitches
from std_srvs.srv import Trigger
from rclpy.task import Future

class DiggerNode(Node):
    def __init__(self):
        """Initialize the ROS 2 digger node."""
        super().__init__("digger")

        # Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(SetPower, "digger/toggle", self.toggle_callback)
        self.srv_stop = self.create_service(Trigger, "digger/stop", self.stop_callback)
        self.srv_setPower = self.create_service(SetPower, "digger/setPower", self.set_power_callback)
        self.srv_setPosition = self.create_service(SetPosition, "lift/setPosition", self.set_position_callback)
        self.srv_lift_stop = self.create_service(Trigger, "lift/stop", self.stop_lift_callback)
        self.srv_lift_set_power = self.create_service(SetPower, "lift/setPower", self.lift_set_power_callback)
        self.srv_zero_lift = self.create_service(Stop, "lift/zero", self.zero_lift_callback)
        self.srv_lower_lift = self.create_service(Stop, "lift/lower", self.lower_lift_callback)

        # Define publishers here

        # Define subscribers here
        self.limit_switch_sub = self.create_subscription(LimitSwitches, "limitSwitches", self.limit_switch_callback, 10)

        # Define default values for our ROS parameters below #
        self.declare_parameter("DIGGER_BELT_MOTOR", 2)
        self.declare_parameter("DIGGER_LIFT_MOTOR", 1)

        # Assign the ROS Parameters to member variables below #
        self.DIGGER_BELT_MOTOR = self.get_parameter("DIGGER_BELT_MOTOR").value
        self.DIGGER_LIFT_MOTOR = self.get_parameter("DIGGER_LIFT_MOTOR").value

        # Print the ROS Parameters to the terminal below #
        self.get_logger().info("DIGGER_BELT_MOTOR has been set to: " + str(self.DIGGER_BELT_MOTOR))
        self.get_logger().info("DIGGER_LIFT_MOTOR has been set to: " + str(self.DIGGER_LIFT_MOTOR))

        self.lift_encoder_offset = 0  # measured in degrees

        # Current state of the digger belt
        self.running = False
        # Current position of the lift motor in degrees
        self.current_position_degrees = 0  # Relative encoders always initialize to 0
        # Current state of the lift system
        self.lift_running = False

        # Limit Switch States
        self.top_limit_pressed = False
        self.bottom_limit_pressed = False
        self.top_limit_event = Future()
        self.bottom_limit_event = Future()


        # Maximum value of the lift motor encoder (bottom of the lift system) IN DEGREES
        self.MAX_ENCODER_DEGREES = (
            -3600 * 360 / 42
        )  # Multiply the max encoder count (-3600) by 360 and divide by 42 to get degrees

    # Define subsystem methods here
    def set_power(self, digger_power: float) -> None:
        """This method sets power to the digger belt."""
        self.running = True
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.DIGGER_BELT_MOTOR, value=digger_power)
        )

    def stop(self) -> None:
        """This method stops the digger belt."""
        self.running = False
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.DIGGER_BELT_MOTOR, value=0.0)
        )

    def toggle(self, digger_belt_power: float) -> None:
        """This method toggles the digger belt."""
        if self.running:
            self.stop()
        else:
            self.set_power(digger_belt_power)


    def stop_lift(self) -> None:
        """This method stops the lift."""
        self.lift_running = False
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(type="duty_cycle", can_id=self.DIGGER_LIFT_MOTOR, value=0.0)
        )

    # Define service callback methods here
    def set_power_callback(self, request, response):
        """This service request sets power to the digger belt."""
        self.set_power(request.power)
        response.success = True
        return response

    def stop_callback(self, request, response):
        """This service request stops the digger belt."""
        self.stop()
        response.success = True
        return response

    def toggle_callback(self, request, response):
        """This service request toggles the digger belt."""
        self.toggle(request.power)
        response.success = True
        return response

    async def set_position_callback(self, request, response):
        """This service request sets the position of the lift.
        TODO: Make sure MotorCommandSet.Request(type="position", 
        is cancellable otherwise this will fail"""
        await self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="position",
                can_id=self.DIGGER_LIFT_MOTOR,
                value=float(request.position + self.lift_encoder_offset),
            )
        )
        response.success = 0  # indicates success
        return response

    def stop_lift_callback(self, request, response):
        """This service request stops the lift system."""
        self.stop_lift()
        response.success = True
        return response
    
    @deprecated
    def lift_set_power_callback(self, request, response):
        """This service request sets power to the digger belt."""
        return response

    async def zero_lift_callback(self, request, response):
        """This service request zeros the lift system."""
        self.lift_set_power(0.05)
        self.top_limit_event = Future()
        self.top_limit_event.add_done_callback(self.stop_lift)
        await self.bottom_limit_event
        return response

    async def lower_lift_callback(self, request, response):
        """This service request reverse-zeros the lift system, putting it at the lowest point"""
        if self.bottom_limit_pressed:
            return response
        self.lift_set_power(-0.05)
        self.bottom_limit_event = Future()
        self.bottom_limit_event.add_done_callback(self.stop_lift)
        await self.bottom_limit_event
        return self.bottom_limit_event.result()

    # Define subscriber callback methods here
    def limit_switch_callback(self, limit_switches_msg):
        """This subscriber callback method is called whenever a message is received on the limitSwitches topic."""
        if self.top_limit_pressed and not self.top_limit_event.done():
            self.top_limit_pressed = limit_switches_msg.top_limit_switch
            self.top_limit_event.set_result(True)
        if self.bottom_limit_pressed and not self.top_limit_event.done():
            self.bottom_limit_pressed = limit_switches_msg.bottom_limit_switch
            self.top_limit_event.set_result(True)


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = DiggerNode()
    node.get_logger().info("Initializing the Digger subsystem!")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    #rclpy.spin(node)
    executor.spin()
    
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    

# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
