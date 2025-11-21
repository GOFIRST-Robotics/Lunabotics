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

        #Position
        self.push_actuator_position = 0

        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.stop_cb_group = MutuallyExclusiveCallbackGroup()

    
        # TODO Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define parameters here
        self.declare_parameter("spin_velocity", 0.0)
        self.declare_parameter("push_actuator_position", 0)
        # TODO: Find real value for this
        self.declare_parameter("MAX_PUSH_ACTUATOR_POSITION", 0)
        self.declare_parameter("MIN_PUSH_ACTUATOR_POSITION", 0)
        self.declare_parameter("MAX_PUSH_ACTUATOR_VELOCITY", 0) # should this be absolute velocity ?
        # Since we only care if the tilt actuator is fully extened or fully retracted then we should only need to care about the speed it moves
        self.declare_parameter("TILT_ACTUATOR_SPEED", 0) # make sure to verify direction of this velocity
        # TODO Get real can ids
        self.declare_parameter("TILT_MOTOR_ID", 0)
        self.declare_parameter("PUSH_MOTOR_ID", 0)
        self.declare_parameter("SPIN_MOTOR_ID", 0)

        # Local variables here
        self.spin_velocity = self.get_parameter("spin_velocity").value
        self.push_actuator_position = self.get_parameter("push_actuator_position").value
        self.MAX_PUSH_ACTUATOR_POSITION = self.get_parameter("MAX_PUSH_ACTUATOR_POSITION").value
        self.MIN_PUSH_ACTUATOR_POSITION = self.get_parameter("MIN_PUSH_ACTUATOR_POSITION").value
        self.MAX_PUSH_ACTUATOR_VELOCITY = self.get_parameter("MAX_PUSH_ACTUATOR_VELOCITY").value
        self.TILT_ACTUATOR_SPEED = self.get_parameter("TILT_ACTUATOR_SPEED")
        self.TILT_MOTOR_ID = self.get_parameter("TILT_MOTOR_ID").value
        self.PUSH_MOTOR_ID = self.get_parameter("PUSH_MOTOR_ID").value
        self.SPIN_MOTOR_ID = self.get_parameter("SPIN_MOTOR_ID").value

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

        self.srv_run_auger_spin = self.create_service(
            Trigger,
            "motor_spin/run",
            self.run_auger_spin_velocity_callback,
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
            Potentiometers, "potentiometers", self.push_actuator_position_callback, 10
        )
        # TODO Define publishers here

        self.get_logger().info("spin_velocity is set to: " + str(self.spin_velocity))

    # Define subsystem methods here

    def set_actuator_tilt_extension(self, tilt: bool) -> None:
        """Set the auger tilt position of the actuator. True for extend, False for retract"""
        if tilt:
            self.get_logger().info("Extending tilt actuator")
        else:
            self.get_logger().info("Retracting tilt actuator")

        speed = self.TILT_ACTUATOR_SPEED * 1 if tilt else -1

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                can_id=self.TILT_MOTOR_ID,
                value=speed
            )
        )

    def stop_actuator_tilt(self) -> None:
        """Stop the auger angular position of the auger motor."""
        self.get_logger().info("Stopping tilt actuator")
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                can_id=self.TILT_MOTOR_ID,
                value=0.0,
            )
        )

    # TODO set position of the actuator that pushes the auger into the ground
    def set_actuator_push_position(self, position: float) -> None:
        """Set the target position of the linear actuator that pushes the auger into the ground."""
        if position > self.MAX_PUSH_ACTUATOR_POSITION or position < self.MIN_PUSH_ACTUATOR_POSITION:
            self.get_logger().warn(f"WARNING: Requested push actuator position is out of range, clamping value; requested: {position}")
            position = max(self.MIN_PUSH_ACTUATOR_POSITION, min(position, self.MAX_PUSH_ACTUATOR_POSITION)) # clamp the value to be within range
        self.get_logger().info("Setting auger push actuator position to: " + str(position))

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="position",
                can_id=self.PUSH_MOTOR_ID,
                value=float(position),
            )
        )

    def set_actuator_push_velocity(self, velocity: float) -> None:
        """Set the target velocity of the linear actuator that pushes the auger into the ground."""
        if abs(velocity) > self.MAX_PUSH_ACTUATOR_VELOCITY:
            self.get_logger().warn(f"WARNING: Requested push actuator velocity is too high, clamping value; requested: {velocity}")
            if velocity < 0:
                velocity = -self.MAX_PUSH_ACTUATOR_VELOCITY
            else:
                velocity = self.MAX_PUSH_ACTUATOR_VELOCITY
        self.get_logger().info("Setting auger push actuator velocity to: " + str(velocity))
        
        self.target_actuator_velocity = velocity
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type = "velocity",
                can_id=self.PUSH_MOTOR_ID, 
                value = float(velocity)
            )
        )

    def stop_actuator_push(self) -> None:
        """Stop the linear actuator that pushes the auger into the ground."""
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                can_id=self.PUSH_MOTOR_ID,
                value=0.0,
            )
        )

    def run_auger_spin_velocity(self) -> None:
        """Set the auger spin velocity of the auger motor."""
        self.get_logger().info("Running auger spin at velocity: " + self.spin_velocity)

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                can_id=self.SPIN_MOTOR_ID,
                value=self.spin_velocity,
            )
        )

    def stop_auger_spin(self) -> None:
        """Stop the auger motor from spinning."""
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                can_id=self.SPIN_MOTOR_ID,
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

    def run_auger_spin_velocity_callback(self, request, response):
        """This service request sets the turn velocity of the auger"""
        self.run_auger_spin_velocity()
        response.success = True
        return response

    def stop_spin_callback(self, request, response):
        """This service request stops the motor that spins the auger."""
        self.stop_auger_spin()
        response.success = True
        return response

    def push_actuator_position_callback(self, msg):
        self.push_actuator_position = msg.data


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
