# This ROS 2 node contains the code for the auger subsystem of the robot
import time
import math

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Import ROS 2 formatted message types

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import (
    MotorCommandSet,
    MotorCommandGet,
    AugerSetPushMotor,
    SetScrewMotorSpeed,
)
from rovr_interfaces.srv import SetExtension
from rovr_interfaces.msg import Potentiometers
from std_srvs.srv import Trigger


class Auger(Node):
    def __init__(self):
        "Initialize the ROS 2 Auger node"
        super().__init__("auger")

        # Callback group for anything that changes motor speeds
        # There should not be a need for a separate one because all services
        # should eventually terminate timely
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # TODO Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define parameters here
        self.declare_parameter("POWER_LIMIT", 1)
        self.declare_parameter("SCREW_SPEED", 4000)
        self.declare_parameter(
            "MAX_SCREW_SPEED", 4_000
        )  # in RPM for both negative and positive direction
        self.declare_parameter("MIN_SCREW_DIG_SPEED", 2_000)
        self.declare_parameter("MAX_SPIN_MOTOR_CURRENT", 0)
        self.declare_parameter("push_motor_position", 0)
        self.declare_parameter("tilt_actuator_position", 0)
        # The error range to consider the current 0
        self.declare_parameter("TILT_ACTUATOR_CURRENT_THRESHOLD", 0)
        # TODO: Find real value for this
        self.declare_parameter("MAX_PUSH_MOTOR_POSITION", 0)
        self.declare_parameter("MIN_PUSH_MOTOR_POSITION", 0)
        self.declare_parameter("DEFAULT_PUSH_MOTOR_SPEED", 0)
        self.declare_parameter("MAX_PUSH_MOTOR_CURRENT", 0)
        self.declare_parameter("PUSH_MOTOR_POS_TOLERANCE", 0)
        # Could potentially be faster
        self.declare_parameter("MAX_RETRACT_PUSH_MOTOR_VELOCITY", -600)
        self.declare_parameter("MAX_EXTEND_PUSH_MOTOR_VELOCITY", 600)
        # Since we only care if the tilt actuator is fully extened or fully
        # retracted then we should only need to care about the speed it moves
        # make sure to verify direction of this velocity
        self.declare_parameter("TILT_ACTUATOR_SPEED", 0)
        # Minimum amount the tilt actuator needs to be extened to safely extend
        # push motor
        self.declare_parameter("TILT_ACTUATOR_MIN_EXTENSION", 0)
        # Minimum amount the push motor needs to be retracted to safely retract
        # tilt actuator
        self.declare_parameter("PUSH_MOTOR_MIN_RETRACTION", 0)
        # TODO Get real can ids
        self.declare_parameter("TILT_ACTUATOR_ID", 0)
        self.declare_parameter("PUSH_MOTOR_ID", 0)
        self.declare_parameter("SPIN_MOTOR_ID", 0)

        # Local variables here
        self.POWER_LIMIT = self.get_parameter("POWER_LIMIT").value
        self.SCREW_SPEED = self.get_parameter("SCREW_SPEED").value
        self.MAX_SCREW_SPEED = self.get_parameter("MAX_SCREW_SPEED").value
        self.MIN_SCREW_DIG_SPEED = self.get_parameter("MIN_SCREW_DIG_SPEED").value
        self.MAX_SPIN_MOTOR_CURRENT = self.get_parameter("MAX_SPIN_MOTOR_CURRENT").value
        self.push_motor_position = self.get_parameter("push_motor_position").value
        self.tilt_actuator_position = self.get_parameter("tilt_actuator_position").value
        self.TILT_ACTUATOR_CURRENT_THRESHOLD = self.get_parameter(
            "TILT_ACTUATOR_CURRENT_THRESHOLD"
        )
        self.MAX_PUSH_MOTOR_POSITION = self.get_parameter(
            "MAX_PUSH_MOTOR_POSITION"
        ).value
        self.MIN_PUSH_MOTOR_POSITION = self.get_parameter(
            "MIN_PUSH_MOTOR_POSITION"
        ).value
        self.DEFAULT_PUSH_MOTOR_SPEED = self.get_parameter(
            "DEFAULT_PUSH_MOTOR_SPEED"
        ).value
        self.MAX_PUSH_MOTOR_CURRENT = self.get_parameter("MAX_PUSH_MOTOR_CURRENT").value
        self.PUSH_MOTOR_POS_TOLERANCE = self.get_parameter(
            "PUSH_MOTOR_POS_TOLERANCE"
        ).value
        self.MAX_RETRACT_PUSH_MOTOR_VELOCITY = self.get_parameter(
            "MAX_RETRACT_PUSH_MOTOR_VELOCITY"
        ).value
        self.MAX_EXTEND_PUSH_MOTOR_VELOCITY = self.get_parameter(
            "MAX_EXTEND_PUSH_MOTOR_VELOCITY"
        ).value
        self.TILT_ACTUATOR_SPEED = self.get_parameter("TILT_ACTUATOR_SPEED").value
        self.TILT_ACTUATOR_MIN_EXTENSION = self.get_parameter(
            "TILT_ACTUATOR_MIN_EXTENSION"
        ).value
        self.PUSH_MOTOR_MIN_RETRACTION = self.get_parameter(
            "PUSH_MOTOR_MIN_RETRACTION"
        ).value
        self.TILT_ACTUATOR_ID = self.get_parameter("TILT_ACTUATOR_ID").value
        self.PUSH_MOTOR_ID = self.get_parameter("PUSH_MOTOR_ID").value
        self.SPIN_MOTOR_ID = self.get_parameter("SPIN_MOTOR_ID").value

        # TODO Define services (methods callable from the outside) here
        self.srv_set_tilt_extension = self.create_service(
            SetExtension,
            "auger/tilt_actuator/setExtension",
            self.set_tilt_extension_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_stop_tilt = self.create_service(
            Trigger,
            "auger/tilt_actuator/stop",
            self.stop_tilt_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_set_push_position = self.create_service(
            AugerSetPushMotor,
            "auger/push_motor/setPosition",
            self.set_push_position_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_stop_push = self.create_service(
            Trigger,
            "auger/push_motor/stop",
            self.stop_push_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_extend_push = self.create_service(
            Trigger,
            "auger/push_motor/extend",
            self.extend_push_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_retract_push = self.create_service(
            Trigger,
            "auger/push_motor/retract",
            self.retract_push_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_run_auger_spin = self.create_service(
            SetScrewMotorSpeed,
            "auger/screw/run",
            self.run_auger_spin_velocity_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_stop_spin = self.create_service(
            Trigger,
            "auger/screw/stop",
            self.stop_spin_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_extend_digger = self.create_service(
            Trigger,
            "auger/control/extend_digger",
            self.extend_digger_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_retract_digger = self.create_service(
            Trigger,
            "auger/control/retract_digger",
            self.retract_digger_callback,
            callback_group=self.service_cb_group,
        )

        self.srv_retract_digger = self.create_service(
            Trigger,
            "auger/control/stop_all",
            self.stop_all,
            callback_group=self.service_cb_group,
        )

        # TODO Define subscribers here - need to subscribe to potentiometer
        # readings?
        self.potentiometer_sub = self.create_subscription(
            Potentiometers, "potentiometer", self.push_motor_position_callback, 10
        )

        # TODO Define publishers here

    # Define subsystem methods here

    def set_actuator_tilt_extension(self, tilt: bool) -> bool:
        """
        Sets the auger tilt position of the actuator. True for extend, False for retract.
        This method will spin until the actuator has hit a limit switch.
        This will return false and do nothing if the push motor is currently extended.
        Caller is responsible for timeouts.
        """
        push_motor_pos_future = self.cli_motor_get.call_async(
            MotorCommandGet.Request(type="position", can_id=self.PUSH_MOTOR_ID)
        )
        rclpy.spin_until_future_complete(self, push_motor_pos_future)
        push_motor_pos = push_motor_pos_future.result()
        if not push_motor_pos.success:
            self.get_logger().info(
                "WARNING: Failed to move the tilt actuator because the push motor position could not be determined"
            )
            return False
        if push_motor_pos.data > self.PUSH_MOTOR_MIN_RETRACTION:
            self.get_logger().info(
                "WARNING: Failed to move the tilt actuator because the push motor is extended too far"
            )
            return False

        if tilt:
            self.get_logger().info("Extending tilt actuator")
        else:
            self.get_logger().info("Retracting tilt actuator")

        speed = self.TILT_ACTUATOR_SPEED * (1 if tilt else -1)

        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity", can_id=self.TILT_ACTUATOR_ID, value=speed
            )
        )
        rclpy.spin_until_future_complete(self, motor_set_future)
        if not motor_set_future.result().success:
            self.get_logger().info("WARNING: Failed to set tilt motor velocity")
            return False

        # gets motor current until it is 0 which means it has hit an limit
        # switch
        while True:
            motor_get_future = self.cli_motor_get.call_async(
                MotorCommandGet.Request(
                    type="current",
                    can_id=self.TILT_ACTUATOR_ID,
                )
            )
            rclpy.spin_until_future_complete(self, motor_get_future)

            if motor_get_future.result().success:
                if (
                    abs(motor_get_future.result().data)
                    < self.TILT_ACTUATOR_CURRENT_THRESHOLD
                ):
                    break
            else:
                self.get_logger().info("WARNING: Failed to read tilt actuator position")

            time.sleep(0.1)

        return True

    def stop_actuator_tilt(self) -> bool:
        """Stop the auger angular position of the auger motor."""
        self.get_logger().info("Stopping tilt actuator")
        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                can_id=self.TILT_ACTUATOR_ID,
                value=0.0,
            )
        )
        rclpy.spin_until_future_complete(self, motor_set_future)
        return motor_set_future.result().success

    def set_motor_push_position(
        self, speed: float, desired_position: float, power_limit: float
    ) -> bool:
        """
        Set the target position of the motor that pushes the auger into the ground.
        This will spin until the motor reaches given setpoint.
        This will fail if the screw is not spinning fast enough
        Caller is responsible for timeouts.
        """
        if self.tilt_actuator_position < self.TILT_ACTUATOR_MIN_EXTENSION:
            self.get_logger().warn(
                "WARNING: Push motor will not move because the tilt actuator is not extended"
            )
            return False

        get_screw_speed_future = self.cli_motor_get.call_async(
            MotorCommandGet.Request(
                type="velocity",
                can_id=self.SPIN_MOTOR_ID,
            )
        )
        rclpy.spin_until_future_complete(self, get_screw_speed_future)

        if not get_screw_speed_future.result().success:
            self.get_logger().warn(
                "WARNING: Push motor will not move because the screw speed could not be determined"
            )
            return False
        elif get_screw_speed_future.result().data < self.MIN_SCREW_DIG_SPEED:
            self.get_logger().warn(
                "WARNING: Push motor will not move because the screw is not spinning fast enough"
            )
            return False

        if (
            desired_position > self.MAX_PUSH_MOTOR_POSITION
            or desired_position < self.MIN_PUSH_MOTOR_POSITION
        ):
            self.get_logger().warn(
                f"WARNING: Requested push motor position is out of range, clamping value; requested: {desired_position}"
            )
            desired_position = max(
                self.MIN_PUSH_MOTOR_POSITION,
                min(desired_position, self.MAX_PUSH_MOTOR_POSITION),
            )  # clamp the value to be within range
        self.get_logger().info(
            "Setting auger push motor position to: " + str(desired_position)
        )

        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                power_limit=power_limit,
                can_id=self.PUSH_MOTOR_ID,
                value=float(speed),
            )
        )
        rclpy.spin_until_future_complete(self, motor_set_future)

        if not motor_set_future.result().success:
            self.get_logger().warn("WARNING: Failed to set push motor voltage")
            return False

        # wait till motor reaches desired position
        while True:
            motor_get_pos_future = self.cli_motor_get.call_async(
                MotorCommandGet.Request(
                    type="position",
                    can_id=self.PUSH_MOTOR_ID,
                )
            )
            rclpy.spin_until_future_complete(self, motor_get_pos_future)

            if motor_get_pos_future.result().success:
                current_pos = motor_get_pos_future.result().data
                if (speed <= 0 and current_pos <= desired_position) or (
                    speed > 0 and current_pos >= desired_position
                ):
                    break
            else:
                self.get_logger().warn("WARNING: Failed to read push motor position")

            time.sleep(0.1)

        stop_motor_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                power_limit=power_limit,
                can_id=self.PUSH_MOTOR_ID,
                value=0,
            )
        )
        stop_motor_response = rclpy.spin_until_future_complete(self, stop_motor_future)
        if not stop_motor_response.result().success:
            self.get_logger().warn("WARNING: Failed to stop the auger screw")
            return False

        return True

    def extend_motor_push(self) -> bool:
        """
        Extends the push motor at max extend speed, returns false
        if it is not able to due to the tilt actuator not being fully extended.
        """
        return self.set_motor_push_position(
            self.DEFAULT_PUSH_MOTOR_SPEED, self.MAX_PUSH_MOTOR_POSITION, 0.5
        )

    def retract_motor_push(self) -> bool:
        """
        Run the push motor at max retract speed.
        Returns false if it is not able to due to the tilt actuator not being fully extended.
        """
        return self.set_motor_push_position(
            -self.DEFAULT_PUSH_MOTOR_SPEED, self.MIN_PUSH_MOTOR_POSITION, 0.5
        )

    def stop_motor_push(self) -> bool:
        """Stop the motor that pushes the auger into the ground."""
        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                can_id=self.PUSH_MOTOR_ID,
                value=0.0,
            )
        )
        rclpy.spin_until_future_complete(self, motor_set_future)
        return motor_set_future.result().success

    def run_auger_spin_velocity(self, desired_speed: float, power_limit: float) -> bool:
        """Set the auger spin velocity of the auger motor."""
        if abs(desired_speed) > self.MAX_SCREW_SPEED:
            self.get_logger().info(
                f"WARNING: Requested auger screw speed is too fast: {desired_speed}"
            )
            # This is the same as signum(desired_speed) * self.MAX_SCREW_SPEED
            desired_speed = math.copysign(self.MAX_SCREW_SPEED, desired_speed)

        self.get_logger().info(f"Running auger spin at velocity: {desired_speed}")

        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                can_id=self.SPIN_MOTOR_ID,
                value=float(desired_speed),
                power_limit=power_limit,
            )
        )
        rclpy.spin_until_future_complete(self, motor_set_future)
        return motor_set_future.result().success

    def stop_auger_spin(self) -> bool:
        """Stop the auger motor from spinning."""
        self.get_logger().info("Stopping auger spin")

        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle",
                can_id=self.SPIN_MOTOR_ID,
                value=0.0,
            )
        )
        rclpy.spin_until_future_complete(self, motor_set_future)
        return motor_set_future.result().success

    def extend_digger(self) -> bool:
        """Tilt and extend"""

        tilt_success = self.set_actuator_tilt_extension(True)
        if not tilt_success:
            return False

        spin_success = self.run_auger_spin_velocity(self.SCREW_SPEED, self.POWER_LIMIT)

        if not spin_success:
            return False

        extend_success = self.extend_motor_push()
        if not extend_success:
            return False

        return True

    def retract_digger(self) -> bool:
        """Tilt and retract"""

        retract_success = self.retract_motor_push()
        if not retract_success:
            return False

        spin_success = self.stop_auger_spin()
        if not spin_success:
            return False

        tilt_success = self.set_actuator_tilt_extension(False)
        if not tilt_success:
            return False

        return True

    def stop_all(self) -> bool:
        # This does not short circit but still returns false if any do
        return (
            self.stop_actuator_tilt() & self.stop_auger_spin() & self.stop_motor_push()
        )

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
        """
        This service request sets position of the motor that pushes the auger into the ground.
        It will fail if the tilt actuator is not fully extended
        """
        response.success = self.set_motor_push_position(
            request.speed, request.position, request.power_limit
        )
        return response

    def stop_push_callback(self, request, response):
        """This service request stops the motor that pushes the auger into the ground."""
        response.success = self.stop_motor_push()
        return response

    def run_auger_spin_velocity_callback(self, request, response):
        """This service request sets the turn velocity of the auger"""
        response.success = self.run_auger_spin_velocity(
            request.speed, request.power_limit
        )
        return response

    def stop_spin_callback(self, request, response):
        """This service request stops the motor that spins the auger."""
        response.success = self.stop_auger_spin()
        return response

    def push_motor_position_callback(self, msg):
        self.tilt_actuator_position = msg.data

    def extend_push_callback(self, request, response):
        """
        This service requests to extend the push motor at full speed.
        It will fail if the tilt actuator is not fully extended
        """
        response.success = self.extend_motor_push()
        return response

    def retract_push_callback(self, request, response):
        """
        This service requests to retract the push motor at full speed.
        It will fail if the tilt actuator is not fully extended
        """
        response.success = self.retract_motor_push()
        return response

    def extend_digger_callback(self, request, response):
        """This service will both tilt and extend the auger."""
        response.success = self.extend_digger()
        return response

    def retract_digger_callback(self, request, response):
        """This service will both tilt and retract the auger."""
        response.success = self.retract_digger()
        return response

    def stop_all_callback(self, request, response):
        """This Service will stop all three motors"""
        response.success = self.stop_all()
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
