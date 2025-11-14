# This ROS 2 node contains the code for the auger subsystem of the robot
import time

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Import ROS 2 formatted message types
from std_msgs.msg import Float32, Float32MultiArray

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import MotorCommandSet, MotorCommandGet
from rovr_interfaces.srv import SetPower, SetPosition, SetExtension
from rovr_interfaces.msg import Potentiometers
from std_srvs.srv import Trigger


class Auger(Node):
    def __init__(self):
        "Initialize the ROS 2 Auger node"
        super().__init__("auger")

        # TODO Get real can ids
        self.TILT_MOTOR_ID = 0
        self.PUSH_MOTOR_ID = 0
        self.SPIN_MOTOR_ID = 0 

        #Position
        self.position = 0

        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.stop_cb_group = MutuallyExclusiveCallbackGroup()

        # Subscriber Stuff

    
        # TODO Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define parameters here
        self.declare_parameter("push_position_limit", 1.0)
        self.declare_parameter("spin_velocity", 0.0)

        # Local variables here
        self.tilt_limit = self.get_parameter("tilt_limit").value
        self.push_position_limit = self.get_parameter("push_position_limit").value
        self.spin_velocity = self.get_parameter("spin_velocity").value

        # TODO Define services (methods callable from the outside) here
        self.srv_set_tilt_extension = self.create_service(
            SetExtension,
            "auger_tilt/setExtension",
            self.set_tilt_extension_callback,
            callback_group=self.service_cb_group,
        )
        self.srv_stop_tilt = self.create_service(
            Trigger,
            "actuator_tilt/stop",
            self.stop_tilt_callback,
            callback_group=self.stop_cb_group,
        )

        self.srv_set_push_position = self.create_service(
            SetPosition,
            "actuator_push/setPosition",
            self.set_push_position_callback,
            callback_group=self.service_cb_group,
        )
        self.srv_stop_push = self.create_service(
            Trigger,
            "actuator_push/stop",
            self.stop_push_callback,
            callback_group=self.stop_cb_group,
        )

        self.srv_set_angular_velocity = self.create_service(
            SetPower,
            "motor_spin/setPower",
            self.set_auger_spin_velocity_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_stop_spin = self.create_service(
            Trigger,
            "motor_spin/stop",
            self.stop_spin_callback,
            callback_group=self.stop_cb_group,
        )

        # TODO Define subscribers here - need to subscribe to potentiometer readings?
        self.potentiometer_sub = self.create_subscription(
            Potentiometers, "potentiometers", self.position_callback, 10
        )
        # TODO Define publishers here

        self.get_logger().info("tilt_limit is set to: " + str(self.tilt_limit))
        self.get_logger().info("push_position_limit is set to: " + str(self.push_position_limit))
        self.get_logger().info("spin_velocity is set to: " + str(self.spin_velocity))

    # Define subsystem methods here

    # TODO set tilt position - is this really an angle? More likely a max position.
    # TODO shouldn't we rather have just the two options exctract fully and retract fully? - introduce two loc vals for max and min position?
    def set_actuator_tilt_extension(self, tilt: bool) -> None:
        """Set the auger tilt position of the actuator."""
        self.get_logger().info("The : " + str(tilt))

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                can_id=self.TILT_MOTOR_ID,
                value=1 if tilt else -1,
            )
        )

    def stop_actuator_tilt(self) -> None:
        """Stop the auger angular position of the auger motor."""
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                value=0.0,
            )
        )

    # TODO set position of the actuator that pushes the auger into the ground
    def set_actuator_push_position(self, position: float, power_limit: float) -> None:
        """Set the target position of the linear actuator that pushes the auger into the ground."""
        self.get_logger().info("Setting actuator position to: " + str(position))
        self.target_actuator_position = position
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="position",
                value=float(position),
            )
        )

    def set_actuator_velocity(self, velocity: float) -> None:

        self.get_logger().info("Setting actuator position to: " + str(velocity))
        self.target_actuator_velocity = velocity
        self.cli_motor_set.call_async(
            MotorCommandSet.Request( type = "velocity", value = float(velocity))
        )

    def stop_actuator_push(self) -> None:
        """Stop the linear actuator that pushes the auger into the ground."""
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                value=0.0,
            )
        )

    # TODO set auger spin velocity
    def set_auger_spin_velocity(self, velocity: float) -> None:
        """Set the auger spin velocity of the auger motor."""
        self.get_logger().info("Setting auger spin velocity to: " + str(velocity))

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                value=float(velocity),
            )
        )

    def stop_auger_spin(self) -> None:
        """Stop the auger motor from spinning."""
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                value=0.0,
            )
        )

    # TODO  Define service callback methods here
    def set_tilt_extension_callback(self, request, response):
        """This service request sets position of the angular motor."""
        self.set_actuator_tilt_extension(request.extension)
        response.success = True
        return response

    def stop_tilt_callback(self, request, response):
        """This service request stops the angular motor."""
        self.stop_actuator_tilt()
        response.success = True
        return response

    def set_push_position_callback(self, request, response):
        """This service request sets position of the actuator that pushes the auger into the ground."""
        self.set_actuator_push_position(request.position, request.power_limit)
        response.success = True
        return response

    def stop_push_callback(self, request, response):
        """This service request stops the actuator that pushes the auger into the ground."""
        self.stop_actuator_push()
        response.success = True
        return response

    def set_auger_spin_velocity_callback(self, request, response):
        """This service request sets the turn velocity of the auger"""
        self.set_auger_spin_velocity(request.power)
        response.success = True
        return response

    def stop_spin_callback(self, request, response):
        """This service request stops the motor that spins the auger."""
        self.stop_auger_spin()
        response.success = True
        return response

    def position_callback(self, msg):
        self.position = msg.data


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = Auger()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    node.get_logger().info("Initializing the Auger subsystem!")
    try:
        executor.spin()
    # TODO this exception needed? Could this ever happen?
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down the Auger subsystem.")
    finally:
        node.destroy_node()

    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
