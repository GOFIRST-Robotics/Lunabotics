# This node contains the main control flow of our robot code.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2022
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: November 2023


# Import the ROS 2 module
import rclpy
import time
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Future
from rclpy.node import Node

# Import ROS 2 formatted message types
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from sensor_msgs.msg import Joy
from action_msgs.msg import GoalStatus
from std_msgs.msg import Float32

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import SetPower, SetPosition
from rovr_interfaces.action import CalibrateFieldCoordinates, AutoDig, AutoOffload
from std_srvs.srv import Trigger

# Import Python Modules
from scipy.spatial.transform import Rotation as R

# Import our logitech gamepad button mappings
from rovr_control import gamepad_constants as bindings

# Uncomment the line below to use the Xbox controller mappings instead
# from rovr_control import xbox_controller_constants as bindings

# GLOBAL VARIABLES #
buttons = [0] * 11  # This is to help with button press detection
# Define the possible states of our robot
states = {"Teleop": 0, "Autonomous": 1}


# Helper Method
def create_pose_stamped(x, y, yaw):
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header.frame_id = "map"
    pose_stamped_msg.pose.position.x = x
    pose_stamped_msg.pose.position.y = y
    quat = R.from_euler("z", yaw, degrees=True).as_quat()
    pose_stamped_msg.pose.orientation.x = quat[0]
    pose_stamped_msg.pose.orientation.y = quat[1]
    pose_stamped_msg.pose.orientation.z = quat[2]
    pose_stamped_msg.pose.orientation.w = quat[3]
    return pose_stamped_msg


class MainControlNode(Node):
    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Autonomous Goal successfully canceled")
            self.end_autonomous()
        else:
            self.get_logger().error("Autonomous Goal failed to cancel")

    def __init__(self) -> None:
        """Initialize the ROS2 Node."""
        super().__init__("rovr_control")

        # Define default values for our ROS parameters below #
        self.declare_parameter("max_drive_power", 1.0)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("max_turn_power", 1.0)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("digger_chain_power", 0.2)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("digger_lift_manual_power_down", 0.12)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("digger_lift_manual_power_up", 0.5)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("lift_digging_start_position", 125.0)  # Measured in encoder counts
        self.declare_parameter("DIGGER_SAFETY_ZONE", 120)  # Measured in potentiometer units (0 to 1023)
        self.declare_parameter("dumper_power", 0.75)  # The power the dumper needs to go
        # The type of field ("cosmic", "top", "bottom", "nasa")
        self.declare_parameter("autonomous_field_type", "cosmic")

        # Assign the ROS Parameters to member variables below #
        self.max_drive_power = self.get_parameter("max_drive_power").value
        self.max_turn_power = self.get_parameter("max_turn_power").value
        self.digger_chain_power = self.get_parameter("digger_chain_power").value
        self.digger_lift_manual_power_down = self.get_parameter("digger_lift_manual_power_down").value
        self.digger_lift_manual_power_up = self.get_parameter("digger_lift_manual_power_up").value
        self.autonomous_field_type = self.get_parameter("autonomous_field_type").value
        self.lift_digging_start_position = self.get_parameter("lift_digging_start_position").value
        self.dumper_power = self.get_parameter("dumper_power").value
        self.DIGGER_SAFETY_ZONE = self.get_parameter("DIGGER_SAFETY_ZONE").value

        # Print the ROS Parameters to the terminal below #
        self.get_logger().info("max_drive_power has been set to: " + str(self.max_drive_power))
        self.get_logger().info("max_turn_power has been set to: " + str(self.max_turn_power))
        self.get_logger().info("digger_chain_power has been set to: " + str(self.digger_chain_power))
        self.get_logger().info(
            "digger_lift_manual_power_down has been set to: " + str(self.digger_lift_manual_power_down)
        )
        self.get_logger().info("digger_lift_manual_power_up has been set to: " + str(self.digger_lift_manual_power_up))
        self.get_logger().info("autonomous_field_type has been set to: " + str(self.autonomous_field_type))
        self.get_logger().info("lift_digging_start_position has been set to: " + str(self.lift_digging_start_position))
        self.get_logger().info("dumper_power has been set to: " + str(self.dumper_power))
        self.get_logger().info("DIGGER_SAFETY_ZONE has been set to: " + str(self.DIGGER_SAFETY_ZONE))

        # Define some initial states here
        self.state = states["Teleop"]

        # Define service clients here
        self.cli_dumper_toggle = self.create_client(Trigger, "dumper/toggle")
        self.cli_dumper_setPower = self.create_client(SetPower, "dumper/setPower")
        self.cli_dumper_stop = self.create_client(Trigger, "dumper/stop")
        self.cli_digger_toggle = self.create_client(SetPower, "digger/toggle")
        self.cli_digger_stop = self.create_client(Trigger, "digger/stop")
        self.cli_digger_setPower = self.create_client(SetPower, "digger/setPower")
        self.cli_lift_setPosition = self.create_client(SetPosition, "lift/setPosition")
        self.cli_drivetrain_stop = self.create_client(Trigger, "drivetrain/stop")
        self.cli_lift_stop = self.create_client(Trigger, "lift/stop")
        self.cli_lift_set_power = self.create_client(SetPower, "lift/setPower")

        # Define publishers and subscribers here
        self.drive_power_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # In order to have actions be cancellable they need to be called in a ReentrantCallbackGroup
        self.joy_subscription = self.create_subscription(
            Joy,
            "joy",
            self.joystick_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.lift_pose_subscription = self.create_subscription(Float32, "lift_pose", self.lift_pose_callback, 10)

        self.act_calibrate_field_coordinates = ActionClient(
            self, CalibrateFieldCoordinates, "calibrate_field_coordinates"
        )
        self.act_auto_dig = ActionClient(self, AutoDig, "auto_dig")
        self.act_auto_offload = ActionClient(self, AutoOffload, "auto_offload")

        self.field_calibrated_handle: ClientGoalHandle = ClientGoalHandle(None, None, None)
        self.auto_dig_handle: ClientGoalHandle = ClientGoalHandle(None, None, None)
        self.auto_offload_handle: ClientGoalHandle = ClientGoalHandle(None, None, None)

        # Current position of the lift motor in potentiometer units (0 to 1023)
        self.current_lift_position = None  # We don't know the current position yet

        # Add watchdog parameters
        self.declare_parameter("watchdog_timeout", 0.5)  # Timeout in seconds
        self.watchdog_timeout = self.get_parameter("watchdog_timeout").value
        self.last_joy_timestamp = time.time()

        # Create timer for watchdog
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)  # Check every 0.1 seconds
        self.connection_active = True

    def stop_all_subsystems(self) -> None:
        """This method stops all subsystems on the robot."""
        self.cli_digger_stop.call_async(Trigger.Request())  # Stop the digger chain
        self.cli_drivetrain_stop.call_async(Trigger.Request())  # Stop the drivetrain
        self.cli_lift_stop.call_async(Trigger.Request())  # Stop the digger lift
        self.cli_dumper_stop.call_async(Trigger.Request())  # Stop the dumper

    def end_autonomous(self) -> None:
        """This method returns to teleop control."""
        self.stop_all_subsystems()  # Stop all subsystems
        self.state = states["Teleop"]  # Return to Teleop mode

    def get_result_callback(self, future: Future):
        goal_handle = future.result()
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Autonomous Goal succeeded!")
            self.end_autonomous()
        else:
            self.get_logger().info("Autonomous Goal failed (or terminated)!")
            self.end_autonomous()

    async def joystick_callback(self, msg: Joy) -> None:
        """This method is called whenever a joystick message is received."""

        # Update watchdog timestamp
        self.last_joy_timestamp = time.time()

        # PUT TELEOP CONTROLS BELOW #

        if self.state == states["Teleop"]:
            # Drive the robot using joystick input during Teleop (Arcade Drive)
            forward_power = msg.axes[bindings.RIGHT_JOYSTICK_VERTICAL_AXIS] * self.max_drive_power  # Forward power
            turning_power = msg.axes[bindings.LEFT_JOYSTICK_HORIZONTAL_AXIS] * self.max_turn_power  # Turning power
            self.drive_power_publisher.publish(Twist(linear=Vector3(x=forward_power), angular=Vector3(z=turning_power)))

            # Check if the digger button is pressed #
            if msg.buttons[bindings.X_BUTTON] == 1 and buttons[bindings.X_BUTTON] == 0:
                self.cli_digger_toggle.call_async(SetPower.Request(power=self.digger_chain_power))

            # Check if the dumper button is pressed #
            if msg.buttons[bindings.B_BUTTON] == 1 and buttons[bindings.B_BUTTON] == 0:
                self.cli_dumper_stop.call_async(Trigger.Request())  # Stop whatever the dumper is doing
                self.cli_dumper_toggle.call_async(Trigger.Request())  # Toggle the dumper (extended or retracted)

            # Manually adjust the dumper position with the left and right bumpers
            if msg.buttons[bindings.RIGHT_BUMPER] == 1 and buttons[bindings.RIGHT_BUMPER] == 0:
                self.cli_dumper_setPower.call_async(SetPower.Request(power=self.dumper_power))
            elif msg.buttons[bindings.RIGHT_BUMPER] == 0 and buttons[bindings.RIGHT_BUMPER] == 1:
                self.cli_dumper_stop.call_async(Trigger.Request())
            elif msg.buttons[bindings.LEFT_BUMPER] == 1 and buttons[bindings.LEFT_BUMPER] == 0:
                self.cli_dumper_setPower.call_async(SetPower.Request(power=-self.dumper_power))
            elif msg.buttons[bindings.LEFT_BUMPER] == 0 and buttons[bindings.LEFT_BUMPER] == 1:
                self.cli_dumper_stop.call_async(Trigger.Request())

            # Manually adjust the height of the digger with the left and right triggers
            if msg.buttons[bindings.LEFT_TRIGGER] == 1 and buttons[bindings.LEFT_TRIGGER] == 0:
                self.cli_lift_set_power.call_async(SetPower.Request(power=self.digger_lift_manual_power_up))
            elif msg.buttons[bindings.LEFT_TRIGGER] == 0 and buttons[bindings.LEFT_TRIGGER] == 1:
                self.cli_lift_stop.call_async(Trigger.Request())
            elif msg.buttons[bindings.RIGHT_TRIGGER] == 1 and buttons[bindings.RIGHT_TRIGGER] == 0:
                if self.current_lift_position and self.current_lift_position < self.DIGGER_SAFETY_ZONE:
                    self.cli_lift_set_power.call_async(SetPower.Request(power=-self.digger_lift_manual_power_up))
                else:
                    self.cli_lift_set_power.call_async(SetPower.Request(power=-self.digger_lift_manual_power_down))
            elif msg.buttons[bindings.RIGHT_TRIGGER] == 0 and buttons[bindings.RIGHT_TRIGGER] == 1:
                self.cli_lift_stop.call_async(Trigger.Request())

        # THE CONTROLS BELOW ALWAYS WORK #

        # Check if the Apriltag calibration button is pressed
        # TODO: This autonomous action needs to be tested on the physical robot!
        if msg.buttons[bindings.A_BUTTON] == 1 and buttons[bindings.A_BUTTON] == 0:
            # Check if the field calibration process is not running
            if self.field_calibrated_handle.status != GoalStatus.STATUS_EXECUTING:
                if not self.act_calibrate_field_coordinates.wait_for_server(timeout_sec=1.0):
                    self.get_logger().error("Field calibration action not available")
                    return
                self.stop_all_subsystems()
                self.field_calibrated_handle = await self.act_calibrate_field_coordinates.send_goal_async(
                    CalibrateFieldCoordinates.Goal()
                )
                if not self.field_calibrated_handle.accepted:
                    self.get_logger().info("Field calibration Goal rejected")
                    return
                self.field_calibrated_handle.get_result_async().add_done_callback(self.get_result_callback)
                self.state = states["Autonomous"]
            # Terminate the field calibration process
            else:
                self.get_logger().warn("Field Calibration Terminated")
                # Cancel the goal
                future = self.field_calibrated_handle.cancel_goal_async()
                future.add_done_callback(self.cancel_done)

        # Check if the autonomous digging button is pressed
        if msg.buttons[bindings.START_BUTTON] == 1 and buttons[bindings.START_BUTTON] == 0:
            # Check if the auto digging process is not running
            if self.auto_dig_handle.status != GoalStatus.STATUS_EXECUTING:
                if not self.act_auto_dig.wait_for_server(timeout_sec=1.0):
                    self.get_logger().error("Auto dig action not available")
                    return
                self.stop_all_subsystems()
                goal = AutoDig.Goal(
                    lift_digging_start_position=self.lift_digging_start_position,
                    digger_chain_power=self.digger_chain_power,
                )
                self.auto_dig_handle = await self.act_auto_dig.send_goal_async(goal)
                self.auto_dig_handle.get_result_async().add_done_callback(self.get_result_callback)
                self.state = states["Autonomous"]
            # Terminate the auto dig process
            else:
                self.get_logger().warn("Auto Dig Terminated")
                # Cancel the goal
                future = self.auto_dig_handle.cancel_goal_async()
                future.add_done_callback(self.cancel_done)

        # Check if the autonomous offload button is pressed
        if msg.buttons[bindings.BACK_BUTTON] == 1 and buttons[bindings.BACK_BUTTON] == 0:
            # Check if the auto offload process is not running
            if self.auto_offload_handle.status != GoalStatus.STATUS_EXECUTING:
                if not self.act_auto_offload.wait_for_server(timeout_sec=1.0):
                    self.get_logger().error("Auto offload action not available")
                    return
                self.stop_all_subsystems()
                goal = AutoOffload.Goal()
                self.auto_offload_handle = await self.act_auto_offload.send_goal_async(goal)
                self.auto_offload_handle.get_result_async().add_done_callback(self.get_result_callback)
                self.state = states["Autonomous"]
            # Terminate the auto offload process
            else:
                self.get_logger().warn("Auto Offload Terminated")
                # Cancel the goal
                future = self.auto_offload_handle.cancel_goal_async()
                future.add_done_callback(self.cancel_done)

        # Update button states (this allows us to detect changing button states)
        for index in range(len(buttons)):
            buttons[index] = msg.buttons[index]

    def watchdog_callback(self):
        """Check if we've received joystick messages recently"""
        current_time = time.time()
        time_since_last_joy = current_time - self.last_joy_timestamp

        # If we haven't received a message in watchdog_timeout seconds
        if time_since_last_joy > self.watchdog_timeout:
            if self.connection_active:  # Only trigger once when connection is lost
                self.get_logger().warn(
                    f"No joystick messages received for {time_since_last_joy:.2f} seconds! Stopping the robot."
                )
                self.stop_all_subsystems()  # Stop all robot movement! (safety feature)
                self.connection_active = False
        elif not self.connection_active:
            self.get_logger().warn("Joystick messages received! Functionality of the robot has been restored.")
            self.connection_active = True

    # Define the subscriber callback for the lift pose topic
    def lift_pose_callback(self, msg: Float32):
        # Average the two potentiometer values
        self.current_lift_position = msg.data


def main(args=None) -> None:
    rclpy.init(args=args)

    main_node = MainControlNode()  # Instantiate the node
    main_node.get_logger().info("Hello from the rovr_control package!")
    rclpy.spin(main_node)  # Spin the node

    # Clean up and shutdown
    main_node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
