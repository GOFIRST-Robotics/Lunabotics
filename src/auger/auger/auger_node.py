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

        # Callback group for anything that changes motor speeds
        # There should not be a need for a separate one because all services should eventually terminate timely
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
    
        # TODO Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define parameters here
        self.declare_parameter("SPIN_VELOCITY", 4_000) # in RPM
        self.declare_parameter("push_motor_position", 0)
        self.declare_parameter("tilt_actuator_position", 0)
        # TODO: Find real value for this
        self.declare_parameter("MAX_PUSH_MOTOR_POSITION", 0)
        self.declare_parameter("MIN_PUSH_MOTOR_POSITION", 0)
        self.declare_parameter("MAX_RETRACT_PUSH_MOTOR_VELOCITY", -600) # Could potentially be faster
        self.declare_parameter("MAX_EXTEND_PUSH_MOTOR_VELOCITY", 600)
        # Since we only care if the tilt actuator is fully extened or fully retracted then we should only need to care about the speed it moves
        self.declare_parameter("TILT_ACTUATOR_SPEED", 0) # make sure to verify direction of this velocity
        self.declare_parameter("TILT_ACTUATOR_MIN_EXTENSION", 0) # Minimum amount the tilt actuator needs to be extened to safely extend push motor
        self.declare_parameter("PUSH_MOTOR_MIN_RETRACTION", 0) # Minimum amount the push motor needs to be retracted to safely retract tilt actuator
        # TODO Get real can ids
        self.declare_parameter("TILT_ACTUATOR_ID", 0)
        self.declare_parameter("PUSH_MOTOR_ID", 0)
        self.declare_parameter("SPIN_MOTOR_ID", 0)

        # Local variables here
        self.SPIN_VELOCITY = self.get_parameter("SPIN_VELOCITY").value
        self.push_motor_position = self.get_parameter("push_motor_position").value
        self.tilt_actuator_position = self.get_parameter("tilt_actuator_position").value
        self.MAX_PUSH_MOTOR_POSITION = self.get_parameter("MAX_PUSH_MOTOR_POSITION").value
        self.MIN_PUSH_MOTOR_POSITION = self.get_parameter("MIN_PUSH_MOTOR_POSITION").value
        self.MAX_RETRACT_PUSH_MOTOR_VELOCITY = self.get_parameter("MAX_RETRACT_PUSH_MOTOR_VELOCITY").value
        self.MAX_EXTEND_PUSH_MOTOR_VELOCITY = self.get_parameter("MAX_EXTEND_PUSH_MOTOR_VELOCITY").value
        self.TILT_ACTUATOR_SPEED = self.get_parameter("TILT_ACTUATOR_SPEED").value
        self.TILT_ACTUATOR_MIN_EXTENSION = self.get_parameter("TILT_ACTUATOR_MIN_EXTENSION").value
        self.PUSH_MOTOR_MIN_RETRACTION = self.get_parameter("PUSH_MOTOR_MIN_RETRACTION").value
        self.TILT_ACTUATOR_ID = self.get_parameter("TILT_ACTUATOR_ID").value
        self.PUSH_MOTOR_ID = self.get_parameter("PUSH_MOTOR_ID").value
        self.SPIN_MOTOR_ID = self.get_parameter("SPIN_MOTOR_ID").value

        # TODO Define services (methods callable from the outside) here
        self.srv_set_tilt_extension = self.create_service(
            SetExtension,
            "actuator_tilt/setExtension",
            self.set_tilt_extension_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_stop_tilt = self.create_service(
            Trigger,
            "actuator_tilt/stop",
            self.stop_tilt_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_set_push_position = self.create_service(
            SetPosition,
            "motor_push/setPosition",
            self.set_push_position_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_stop_push = self.create_service(
            Trigger,
            "motor_push/stop",
            self.stop_push_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_extend_push = self.create_service(
            Trigger,
            "motor_push/extend",
            self.extend_push_callback,
            callback_group=self.service_cb_group
        )

        self.srv_retract_push = self.create_service(
            Trigger,
            "motor_push/retract",
            self.retract_push_callback,
            callback_group=self.service_cb_group
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
            callback_group=self.service_cb_group,
        )

        # TODO Define subscribers here - need to subscribe to potentiometer readings?
        self.potentiometer_sub = self.create_subscription(
            Potentiometers, "potentiometers", self.push_motor_position_callback, 10
        )

        # TODO Define publishers here


    # Define subsystem methods here

    def set_actuator_tilt_extension(self, tilt: bool) -> bool:
        """Set the auger tilt position of the actuator. True for extend, False for retract.
        This will return false and do nothing if the push motor is currently extended."""
        # push_motor_pos = self.cli_motor_get.send_request("position", self.PUSH_MOTOR_ID)
        push_motor_pos_future = self.cli_motor_get.call_async(MotorCommandGet.Request(type="position", can_id=self.PUSH_MOTOR_ID))
        rclpy.spin_until_future_complete(self, push_motor_pos_future)
        push_motor_pos = push_motor_pos_future.result()
        if not push_motor_pos.success:
            self.get_logger().info("Failed to move the tilt actuator because the push motor position could not be determined")
            return False
        if push_motor_pos.data > self.PUSH_MOTOR_MIN_RETRACTION:
            self.get_logger().info("Failed to move the tilt actuator because the push motor is extended too far")
            return False

        if tilt:
            self.get_logger().info("Extending tilt actuator")
        else:
            self.get_logger().info("Retracting tilt actuator")

        speed = self.TILT_ACTUATOR_SPEED * (1 if tilt else -1)

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                can_id=self.TILT_ACTUATOR_ID,
                value=speed
            )
        )

    def stop_actuator_tilt(self) -> None:
        """Stop the auger angular position of the auger motor."""
        self.get_logger().info("Stopping tilt actuator")
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                can_id=self.TILT_ACTUATOR_ID,
                value=0.0,
            )
        )

    def set_motor_push_position(self, position: float, power_limit: float) -> bool:
        """Set the target position of the motor that pushes the auger into the ground."""
        if self.tilt_actuator_position < self.TILT_ACTUATOR_MIN_EXTENSION:
            self.get_logger().warn("WARNING: Push motor will not move because the tilt actuator is not extended")
            return False
        if position > self.MAX_PUSH_MOTOR_POSITION or position < self.MIN_PUSH_MOTOR_POSITION:
            self.get_logger().warn(f"WARNING: Requested push motor position is out of range, clamping value; requested: {position}")
            position = max(self.MIN_PUSH_MOTOR_POSITION, min(position, self.MAX_PUSH_MOTOR_POSITION)) # clamp the value to be within range
        self.get_logger().info("Setting auger push motor position to: " + str(position))

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="position",
                power_limit=power_limit,
                can_id=self.PUSH_MOTOR_ID,
                value=float(position),
            )
        )
        return True

    def extend_motor_push(self) -> bool:
        """Extends the push motor at max extend speed, returns false if it is not able to due to the tilt actuator not being fully extended."""
        return self.set_motor_push_velocity(self.MAX_EXTEND_PUSH_MOTOR_VELOCITY)

    def retract_motor_push(self) -> bool:
        """Run the push motor at max retract speed. Returns false if it is not able to due to the tilt actuator not being fully extended."""
        return self.set_motor_push_velocity(self.MAX_RETRACT_PUSH_MOTOR_VELOCITY)

    def set_motor_push_velocity(self, velocity: float) -> bool:
        """Set the target velocity of the motor that pushes the auger into the ground. Returns false if it is not able to due to the tilt actuator not being fully extended."""
        if self.tilt_actuator_position < self.TILT_ACTUATOR_MIN_EXTENSION:
            self.get_logger().warn("WARNING: Push motor will not move because the tilt actuator is not extended")
            return False
        if velocity > self.MAX_EXTEND_PUSH_MOTOR_VELOCITY or velocity < self.MAX_RETRACT_PUSH_MOTOR_VELOCITY:
            self.get_logger().warn(f"WARNING: Requested push motor velocity is too fast, clamping value; requested: {velocity}")
            velocity = min(self.MAX_EXTEND_PUSH_MOTOR_VELOCITY, max(self.MAX_RETRACT_PUSH_MOTOR_VELOCITY, velocity)) # clamp value
        self.get_logger().info("Setting auger push motor velocity to: " + str(velocity))
        
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type = "velocity",
                can_id=self.PUSH_MOTOR_ID, 
                value = float(velocity)
            )
        )
        return True

    def stop_motor_push(self) -> bool:
        """Stop the motor that pushes the auger into the ground."""
        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                can_id=self.PUSH_MOTOR_ID,
                value=0.0,
            )
        )
        return True

    def run_auger_spin_velocity(self) -> bool:
        """Set the auger spin velocity of the auger motor."""
        self.get_logger().info(f"Running auger spin at velocity: {self.SPIN_VELOCITY}")

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                can_id=self.SPIN_MOTOR_ID,
                value=float(self.SPIN_VELOCITY),
            )
        )
        return True

    def stop_auger_spin(self) -> bool:
        """Stop the auger motor from spinning."""
        self.get_logger().info("Stopping auger spin")

        self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                can_id=self.SPIN_MOTOR_ID,
                value=0.0,
            )
        )
        return True

    # TODO  Define service callback methods here
    def set_tilt_extension_callback(self, request, response):
        """This service request sets position of the angular motor."""
        response.success = self.set_actuator_tilt_extension(request.extension)
        return response

    def stop_tilt_callback(self, request, response):
        """This service request stops the angular motor."""
        self.stop_actuator_tilt()
        response.success = True
        return response

    def set_push_position_callback(self, request, response):
        """This service request sets position of the motor that pushes the auger into the ground. It will fail if the tilt actuator is not fully extended"""
        response.success = self.set_motor_push_position(request.position, request.power_limit)
        return response

    def stop_push_callback(self, request, response):
        """This service request stops the motor that pushes the auger into the ground."""
        response.success = self.stop_motor_push()
        return response

    def run_auger_spin_velocity_callback(self, request, response):
        """This service request sets the turn velocity of the auger"""
        response.success = self.run_auger_spin_velocity()
        return response

    def stop_spin_callback(self, request, response):
        """This service request stops the motor that spins the auger."""
        response.success = self.stop_auger_spin()
        return response

    def push_motor_position_callback(self, msg):
        self.tilt_actuator_position = msg.data

    def extend_push_callback(self, request, response):
        """This service requests to extend the push motor at full speed. It will fail if the tilt actuator is not fully extended"""
        response.success = self.extend_motor_push()
        return response

    def retract_push_callback(self, request, response):
        """This service requests to retract the push motor at full speed. It will fail if the tilt actuator is not fully extended"""
        response.success = self.retract_motor_push()
        return response


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
