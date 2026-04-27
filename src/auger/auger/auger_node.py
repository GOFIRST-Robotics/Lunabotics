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
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, Float32


class Auger(Node):
    def __init__(self):
        "Initialize the ROS 2 Auger node"
        super().__init__("auger")

        # Callback group for anything that changes motor speeds
        # There should not be a need for a separate one because all services
        # should eventually terminate timely
        self.stop_service_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # TODO Define service clients here
        self.cli_motor_set = self.create_client(MotorCommandSet, "motor/set")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")

        # Define parameters here
        self.declare_parameter("AUGER_STOWED", True)
        self.declare_parameter("DUMPER_STOWED", True)
        self.declare_parameter("extension_limit_switch", True)
        self.declare_parameter("POWER_LIMIT", 1)
        self.declare_parameter("SCREW_SPEED", 4000)
        self.declare_parameter(
            "MAX_SCREW_SPEED", 4_000
        )  # in RPM for both negative and positive direction
        self.declare_parameter("MIN_SCREW_DIG_SPEED", 2000)
        self.declare_parameter("MAX_SPIN_MOTOR_CURRENT", 0)
        self.declare_parameter("push_motor_position", 0)
        # The error range to consider the current 0
        self.declare_parameter("TILT_ACTUATOR_CURRENT_THRESHOLD", 0.21)
        # TODO: Find real value for this
        self.declare_parameter("MAX_PUSH_MOTOR_POSITION", 10000)
        self.declare_parameter("MIN_PUSH_MOTOR_POSITION", 1000)
        self.declare_parameter("DEFAULT_PUSH_MOTOR_SPEED", 3000)
        self.declare_parameter("MAX_PUSH_MOTOR_CURRENT", 0)
        self.declare_parameter("PUSH_MOTOR_POS_TOLERANCE", 0)
        # Could potentially be faster
        self.declare_parameter("MAX_RETRACT_PUSH_MOTOR_VELOCITY", -600)
        self.declare_parameter("MAX_EXTEND_PUSH_MOTOR_VELOCITY", 600)
        # Since we only care if the tilt actuator is fully extened or fully
        #m retracted then we should only need to care about the speed it moves
        # make sure to verify direction of this velocity
        self.declare_parameter("TILT_ACTUATOR_SPEED", 1.0)
        # Minimum amount the tilt actuator needs to be extened to safely extend
        # push motor
        self.declare_parameter("TILT_ACTUATOR_MIN_EXTENSION", 0)
        # Minimum amount the push motor needs to be retracted to safely retract
        # tilt actuator
        self.declare_parameter("PUSH_MOTOR_MIN_RETRACTION", 0)
        # TODO Get real can ids
        self.declare_parameter("TILT_ACTUATOR_ID", 1)
        self.declare_parameter("PUSH_MOTOR_ID", 0)
        self.declare_parameter("SPIN_MOTOR_ID", 0)
        self.declare_parameter("FAST_SCREW_SPEED", 8000)
        self.declare_parameter("SLOW_SCREW_SPEED", 0)

        # Local variables here
        self.MIN_SCREW_DIG_SPEED = self.get_parameter("MIN_SCREW_DIG_SPEED").value
        self.POWER_LIMIT = self.get_parameter("POWER_LIMIT").value
        self.FAST_SCREW_SPEED = self.get_parameter("FAST_SCREW_SPEED").value
        self.SLOW_SCREW_SPEED = self.get_parameter("SLOW_SCREW_SPEED").value
        self.MAX_SPIN_MOTOR_CURRENT = self.get_parameter("MAX_SPIN_MOTOR_CURRENT").value
        self.push_motor_position = self.get_parameter("push_motor_position").value
        self.extension_limit_switch = self.get_parameter("extension_limit_switch").value
        self.auger_stowed = self.get_parameter("AUGER_STOWED").value
        self.dumper_stowed = self.get_parameter("DUMPER_STOWED").value
        self.TILT_ACTUATOR_CURRENT_THRESHOLD = self.get_parameter(
            "TILT_ACTUATOR_CURRENT_THRESHOLD"
        ).value
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

        self.linear_actuator_tilted: bool = False
        self.cancel_linear_actuator: bool = False
        self.linear_actuator_running: bool = False

        self.cancel_motor_push: bool = False
        self.motor_push_running: bool = False
        
        

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
            callback_group=self.stop_service_cb_group,
        )

        # self.srv_set_push_position = self.create_service(
        #     AugerSetPushMotor,
        #     "auger/push_motor/setPosition",
        #     self.set_push_position_callback,
        #     callback_group=self.service_cb_group,
        # )

        self.srv_stop_push = self.create_service(
            Trigger,
            "auger/push_motor/stop",
            self.stop_push_callback,
            callback_group=self.stop_service_cb_group,
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
            callback_group=self.stop_service_cb_group,
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

        self.srv_stop_all = self.create_service(
            Trigger,
            "auger/control/stop_all",
            self.stop_all_callback,
            callback_group=self.stop_service_cb_group,
        )


        self.limit_switch_sub = self.create_subscription(
            Bool, "ExtensionLimitSwitch", self.limit_switch_callback, 10
        )

        self.dumper_stowed_sub = self.create_subscription(
            Bool, "dumper_stowed", self.dumper_stowed_callback, 10
        )

        # TODO Define publishers here
        self.auger_stowed_pub = self.create_publisher(Bool, "auger_stowed", 10)

        self.extension_pos_pub = self.create_publisher(Float32, "extension_pos", 10)

    # Define subsystem methods here

    def set_actuator_tilt_extension(self, tilt: bool) -> bool:
        """
        Sets the auger tilt position of the actuator. True for extend, False for retract.
        This method will spin until the actuator has hit a limit switch.
        This will return false and do nothing if the push motor is currently extended.
        Caller is responsible for timeouts.
        """
        self.linear_actuator_running = True
        # push_motor_pos_future = self.cli_motor_get.call_async(
        #     MotorCommandGet.Request(type="position", can_id=self.PUSH_MOTOR_ID)
        # )
        # rclpy.spin_until_future_complete(self, push_motor_pos_future)
        # push_motor_pos = push_motor_pos_future.result()
        # if not push_motor_pos.success:
        #     self.get_logger().info(
        #         "WARNING: Failed to move the tilt actuator because the push motor position could not be determined"
        #     )
        #     self.linear_actuator_running = False
        #     return False
        # if push_motor_pos.data > self.PUSH_MOTOR_MIN_RETRACTION:
        #     self.get_logger().info(
        #         "WARNING: Failed to move the tilt actuator because the push motor is extended too far"
        #     )
        #     self.linear_actuator_running = False
        #     return False

        if tilt:
            self.get_logger().info("Extending tilt actuator")
        else:
            self.get_logger().info("Retracting tilt actuator")

        speed = self.TILT_ACTUATOR_SPEED * (1 if tilt else -1)

        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle", can_id=self.TILT_ACTUATOR_ID, value=float(speed)
            )
        )
        
        rclpy.spin_until_future_complete(self, motor_set_future)
        if not motor_set_future.result().success:
            self.get_logger().info("WARNING: Failed to set tilt motor velocity")
            self.linear_actuator_running = False
            return False
        
        # gets motor current until it is 0 which means it has hit an limit
        # switch
        time.sleep(1.5)
        if tilt:
            self.auger_stowed = False
            msg = Bool()
            msg.data = self.auger_stowed
            self.auger_stowed_pub.publish(msg)

        while True:
            motor_get_future = self.cli_motor_get.call_async(
                MotorCommandGet.Request(
                    type="current",
                    can_id=self.TILT_ACTUATOR_ID,
                )
            )
            rclpy.spin_until_future_complete(self, motor_get_future)
            if motor_get_future.result().success:
                # self.get_logger().info(f"Actuator current: {motor_get_future.result().data}")
                if (
                    abs(motor_get_future.result().data)
                    < self.TILT_ACTUATOR_CURRENT_THRESHOLD
                ):
                    break
            else:
                self.get_logger().info("WARNING: Failed to read tilt actuator position")

            time.sleep(0.1)
            
            if self.cancel_linear_actuator:
                self.get_logger().info("Cancelling the linear actuator")
                break

        
        motor_set_future_stop = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="duty_cycle", can_id=self.TILT_ACTUATOR_ID, value=0.0
            )
        )
        rclpy.spin_until_future_complete(self, motor_set_future_stop)
        self.linear_actuator_running = False
        if not motor_set_future_stop.result().success:
            self.get_logger().info("WARNING: Failed to stop tilt motor")
            return False
        if not tilt:
            self.auger_stowed = True
            msg = Bool()
            msg.data = self.auger_stowed
            self.auger_stowed_pub.publish(msg)
        
        self.linear_actuator_tilted = tilt

        return True

    def stop_actuator_tilt(self) -> bool:
        """Stop the auger angular position of the auger motor."""
        self.get_logger().info("In the stop acutator tilt function")
        if self.linear_actuator_running:
            self.cancel_linear_actuator = True
            return True

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

    def set_motor_push_extend(self) -> bool:
        """
        Set the target position of the motor that pushes the auger into the ground.
        This will spin until the motor reaches given setpoint.
        This will fail if the screw is not spinning fast enough
        Caller is responsible for timeouts.
        """
        self.motor_push_running = True
        power_limit = 0.5
        if not self.linear_actuator_tilted:
            self.get_logger().warn(
                "WARNING: Push motor will not move because the tilt actuator is not extended"
            )
            self.motor_push_running = False
            return False

        # get_screw_speed_future = self.cli_motor_get.call_async(
        #     MotorCommandGet.Request(
        #         type="velocity",
        #         can_id=self.SPIN_MOTOR_ID,
        #     )
        # )
        # rclpy.spin_until_future_complete(self, get_screw_speed_future)

        # if not get_screw_speed_future.result().success:
        #     self.get_logger().warn(
        #         "WARNING: Push motor will not move because the screw speed could not be determined"
        #     )
        #     self.motor_push_running = False
        #     return False
        
        # elif get_screw_speed_future.result().data < self.MIN_SCREW_DIG_SPEED:
        #     self.get_logger().warn(
        #         "WARNING: Push motor will not move because the screw is not spinning fast enough"
        #     )
        #     self.motor_push_running = False
        #     return False

        if (
            self.MAX_PUSH_MOTOR_POSITION > self.MAX_PUSH_MOTOR_POSITION
            or self.MAX_PUSH_MOTOR_POSITION < self.MIN_PUSH_MOTOR_POSITION
        ):
            self.get_logger().warn(
                f"WARNING: Requested push motor position is out of range, clamping value; requested: {self.MAX_PUSH_MOTOR_POSITION}"
            )
            self.MAX_PUSH_MOTOR_POSITION = max(
                self.MIN_PUSH_MOTOR_POSITION,
                min(self.MAX_PUSH_MOTOR_POSITION, self.MAX_PUSH_MOTOR_POSITION),
            )  # clamp the value to be within range
        self.get_logger().info(
            "Setting auger push motor position to: " + str(self.MAX_PUSH_MOTOR_POSITION)
        )
        

        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                can_id=self.PUSH_MOTOR_ID,
                value=float(self.DEFAULT_PUSH_MOTOR_SPEED),
            )
        )
        self.get_logger().info(
            f"Set the plunge velocity to {self.DEFAULT_PUSH_MOTOR_SPEED}"
        )
        rclpy.spin_until_future_complete(self, motor_set_future)

        if not motor_set_future.result().success:
            self.get_logger().warn("WARNING: Failed to set push motor voltage")
            self.motor_push_running = False
            return False

        # wait till motor reaches desired position
        while True:
            if self.cancel_motor_push:
                break
            self.get_logger().info(
                 f"getting the plunge velocity"
            )
            motor_get_pos_future = self.cli_motor_get.call_async(
                MotorCommandGet.Request(
                    type="position",
                    can_id=self.PUSH_MOTOR_ID,
                )
            )
            rclpy.spin_until_future_complete(self, motor_get_pos_future)

            if motor_get_pos_future.result().success:
                current_pos = motor_get_pos_future.result().data
                msg = Float32()
                msg.data = current_pos
                self.extension_pos_pub.publish(msg)
                if (
                    (self.DEFAULT_PUSH_MOTOR_SPEED <= 0 and current_pos <= self.MIN_PUSH_MOTOR_POSITION)
                    or (self.DEFAULT_PUSH_MOTOR_SPEED > 0 and current_pos >= self.MAX_PUSH_MOTOR_POSITION)
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
        self.motor_push_running = False
        self.cancel_motor_push = False
        stop_motor_response = rclpy.spin_until_future_complete(self, stop_motor_future)
        if not stop_motor_response.result().success:
            self.get_logger().warn("WARNING: Failed to stop the auger screw")
            return False

        
        return True

    def set_motor_push_retract(self) -> bool:
        """
        Set the target position of the motor that pushes the auger into the ground.
        This will spin until the motor reaches given setpoint.
        This will fail if the screw is not spinning fast enough
        Caller is responsible for timeouts.
        """
        self.motor_push_running = True
        power_limit = 0.5

        if (
            self.MIN_PUSH_MOTOR_POSITION > self.MIN_PUSH_MOTOR_POSITION
            or self.MIN_PUSH_MOTOR_POSITION < self.MIN_PUSH_MOTOR_POSITION
        ):
            self.get_logger().warn(
                f"WARNING: Requested push motor position is out of range, clamping value; requested: {self.MIN_PUSH_MOTOR_POSITION}"
            )
            self.MIN_PUSH_MOTOR_POSITION = max(
                self.MIN_PUSH_MOTOR_POSITION,
                min(self.MIN_PUSH_MOTOR_POSITION, self.MIN_PUSH_MOTOR_POSITION),
            )  # clamp the value to be within range
        self.get_logger().info(
            "Setting auger push motor position to: " + str(self.MIN_PUSH_MOTOR_POSITION)
        )

        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                power_limit=power_limit,
                can_id=self.PUSH_MOTOR_ID,
                value=float(-self.DEFAULT_PUSH_MOTOR_SPEED),
            )
        )
        rclpy.spin_until_future_complete(self, motor_set_future)

        if not motor_set_future.result().success:
            self.get_logger().warn("WARNING: Failed to set push motor voltage")
            self.motor_push_running = False
            return False

        # wait till motor reaches desired position
        while True:
            if self.cancel_motor_push:
                break
                
            motor_get_pos_future = self.cli_motor_get.call_async(
                MotorCommandGet.Request(
                    type="position",
                    can_id=self.PUSH_MOTOR_ID,
                )
            )
            rclpy.spin_until_future_complete(self, motor_get_pos_future)

            if motor_get_pos_future.result().success:
                current_pos = motor_get_pos_future.result().data
                msg = Float32()
                msg.data = current_pos
                self.extension_pos_pub.publish(msg)
                if (
                    (-self.DEFAULT_PUSH_MOTOR_SPEED <= 0 and current_pos <= self.MIN_PUSH_MOTOR_POSITION)
                    or (-self.DEFAULT_PUSH_MOTOR_SPEED > 0 and current_pos >= self.MAX_PUSH_MOTOR_POSITION)
                    or (self.extension_limit_switch)
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
        self.motor_push_running = False
        self.cancel_motor_push = False
        stop_motor_response = rclpy.spin_until_future_complete(self, stop_motor_future)
        if not stop_motor_response.result().success:
            self.get_logger().warn("WARNING: Failed to stop the auger screw")
            return False

        return True

    def stop_motor_push(self) -> bool:
        """Stop the motor that pushes the auger into the ground."""
        if self.motor_push_running:
            self.cancel_motor_push = True
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
        if desired_speed < 0:
            self.get_logger().info(
                f"WARNING: Requested auger screw speed backwards, instead setting to 0"
            )
        
            desired_speed = 0
        elif desired_speed > self.FAST_SCREW_SPEED:
            self.get_logger().info(
                f"WARNING: Requested auger screw speed is too fast, clamping it"
            )
            desired_speed = self.FAST_SCREW_SPEED


        self.get_logger().info(f"Running auger spin at velocity: {desired_speed}")

        motor_set_future = self.cli_motor_set.call_async(
            MotorCommandSet.Request(
                type="velocity",
                can_id=self.SPIN_MOTOR_ID,
                value=float(desired_speed),
                power_limit=float(power_limit),
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
        if not self.dumper_stowed:
            return False

        tilt_success = self.set_actuator_tilt_extension(True)
        
        if not tilt_success or self.cancel_linear_actuator:
            self.cancel_linear_actuator = False
            return False

        spin_success = self.run_auger_spin_velocity(self.FAST_SCREW_SPEED, self.POWER_LIMIT)

        if not spin_success:
            return False

        extend_success = self.set_motor_push_extend()
        if not extend_success:
            return False

        return True

    def retract_digger(self) -> bool:
        """Tilt and retract"""

        retract_success = self.set_motor_push_retract()
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
        self.get_logger().info("in the stop all function")
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

    # def set_push_position_callback(self, request, response):
    #     """
    #     This service request sets position of the motor that pushes the auger into the ground.
    #     It will fail if the tilt actuator is not fully extended
    #     """
    #     response.success = self.set_motor_push_position(
    #         request.speed, request.position, request.power_limit
    #     )
    #     return response

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

    def limit_switch_callback(self, msg):
        self.extension_limit_switch = msg.data

    def dumper_stowed_callback(self, msg):
        self.dumper_stowed = msg.data

    def extend_push_callback(self, request, response):
        """
        This service requests to extend the push motor at full speed.
        It will fail if the tilt actuator is not fully extended
        """
        response.success = self.set_motor_push_extend()
        return response

    def retract_push_callback(self, request, response):
        """
        This service requests to retract the push motor at full speed.
        It will fail if the tilt actuator is not fully extended
        """
        response.success = self.set_motor_push_retract()
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
        self.get_logger().info("In the stop all callback")
        response.success = self.stop_all()
        return response


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = Auger()

    node.get_logger().info("Initializing the Auger subsystem!")
    rclpy.spin(node)


    node.get_logger().info(" subsystem!")

    node.destroy_node()
    rclpy.shutdown()

# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
