# This node contains the main control flow of our robot code.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2022
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: November 2023


# Import the ROS 2 module
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.client import Future
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Provides a “navigation as a library” capability
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
)

# Import ROS 2 formatted message types
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import Stop, Drive, MotorCommandGet, SetPower, SetPosition
from rovr_interfaces.action import CalibrateFieldCoordinates, AutoDig, AutoOffload

# Import Python Modules
import asyncio  # Allows the use of asynchronous methods!
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
    def __init__(self) -> None:
        """Initialize the ROS2 Node."""
        super().__init__("rovr_control")

        # Define default values for our ROS parameters below #
        self.declare_parameter("autonomous_driving_power", 0.25)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("max_drive_power", 1.0)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("max_turn_power", 1.0)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("skimmer_belt_power", -0.3)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("autonomous_field_type", "bottom")  # The type of field ("top", "bottom", "nasa")
        self.declare_parameter("skimmer_lift_manual_power", 0.075)  # Measured in Duty Cycle (0.0-1.0)
        self.declare_parameter("lift_dumping_position", -1000)  # Measured in encoder counts
        self.declare_parameter("lift_digging_start_position", -3050)  # Measured in encoder counts
        self.declare_parameter("lift_digging_end_position", -3150)  # Measured in encoder counts

        # Assign the ROS Parameters to member variables below #
        self.autonomous_driving_power = self.get_parameter("autonomous_driving_power").value
        self.max_drive_power = self.get_parameter("max_drive_power").value
        self.max_turn_power = self.get_parameter("max_turn_power").value
        self.skimmer_belt_power = self.get_parameter("skimmer_belt_power").value
        self.skimmer_lift_manual_power = self.get_parameter("skimmer_lift_manual_power").value
        self.autonomous_field_type = self.get_parameter("autonomous_field_type").value
        self.lift_dumping_position = (
            self.get_parameter("lift_dumping_position").value * 360 / 42
        )  # Convert encoder counts to degrees
        self.lift_digging_start_position = (
            self.get_parameter("lift_digging_start_position").value * 360 / 42
        )  # Convert encoder counts to degrees
        self.lift_digging_end_position = (
            self.get_parameter("lift_digging_end_position").value * 360 / 42
        )  # Convert encoder counts to degrees

        # Print the ROS Parameters to the terminal below #
        self.get_logger().info("autonomous_driving_power has been set to: " + str(self.autonomous_driving_power))
        self.get_logger().info("max_drive_power has been set to: " + str(self.max_drive_power))
        self.get_logger().info("max_turn_power has been set to: " + str(self.max_turn_power))
        self.get_logger().info("skimmer_belt_power has been set to: " + str(self.skimmer_belt_power))
        self.get_logger().info("skimmer_lift_manual_power has been set to: " + str(self.skimmer_lift_manual_power))
        self.get_logger().info("autonomous_field_type has been set to: " + str(self.autonomous_field_type))
        self.get_logger().info("lift_dumping_position has been set to: " + str(self.lift_dumping_position))
        self.get_logger().info("lift_digging_start_position has been set to: " + str(self.lift_digging_start_position))
        self.get_logger().info("lift_digging_end_position has been set to: " + str(self.lift_digging_end_position))

        # Define some initial states here
        self.state = states["Teleop"]
        self.camera_view_toggled = False
        self.front_camera = None
        self.back_camera = None
        self.autonomous_digging_process = None
        self.autonomous_offload_process = None
        self.autonomous_cycle_process = None
        self.skimmer_goal_reached = True

        self.DANGER_THRESHOLD = 1
        self.REAL_DANGER_THRESHOLD = 100

        # Define important map locations
        if self.autonomous_field_type == "top":
            self.autonomous_berm_location = create_pose_stamped(
                7.25, -3.2, 90
            )  # TODO: Test this location in simulation
            self.dig_location = create_pose_stamped(6.2, -1.2, 0)
        elif self.autonomous_field_type == "bottom":
            self.autonomous_berm_location = create_pose_stamped(7.25, -1.4, 270)
            self.dig_location = create_pose_stamped(6.2, -3.2, 270)
        elif self.autonomous_field_type == "nasa":
            self.autonomous_berm_location = create_pose_stamped(1.3, -0.6, 90)  # TODO: Test this location in simulation
            self.dig_location = create_pose_stamped(6.2, -1.2, 0)  # TODO: Test this location in simulation

        # Define service clients here
        self.cli_skimmer_toggle = self.create_client(SetPower, "skimmer/toggle")
        self.cli_skimmer_stop = self.create_client(Stop, "skimmer/stop")
        self.cli_skimmer_setPower = self.create_client(SetPower, "skimmer/setPower")
        self.cli_lift_setPosition = self.create_client(SetPosition, "lift/setPosition")
        self.cli_drivetrain_stop = self.create_client(Stop, "drivetrain/stop")
        self.cli_drivetrain = self.create_client(Drive, "drivetrain/drive")
        self.cli_drivetrain_calibrate = self.create_client(Stop, "drivetrain/calibrate")
        self.cli_motor_get = self.create_client(MotorCommandGet, "motor/get")
        self.cli_lift_stop = self.create_client(Stop, "lift/stop")
        self.cli_lift_zero = self.create_client(Stop, "lift/zero")
        self.cli_lift_set_power = self.create_client(SetPower, "lift/setPower")

        # Define publishers and subscribers here
        self.drive_power_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.joy_subscription = self.create_subscription(Joy, "joy", self.joystick_callback, 10)
        self.skimmer_goal_subscription = self.create_subscription(
            Bool, "/skimmer/goal_reached", self.skimmer_goal_callback, 10
        )

        self.act_calibrate_field_coordinates = ActionClient(
            self, CalibrateFieldCoordinates, "calibrate_field_coordinates"
        )
        self.act_auto_dig = ActionClient(self, AutoDig, "auto_dig")
        self.act_auto_offload = ActionClient(self, AutoOffload, "auto_offload")

        self.field_calibrated_handle: ClientGoalHandle = ClientGoalHandle(None, None, None)
        self.auto_dig_handle: ClientGoalHandle = ClientGoalHandle(None, None, None)
        self.auto_offload_handle: ClientGoalHandle = ClientGoalHandle(None, None, None)

        self.nav2 = BasicNavigator()  # Instantiate the BasicNavigator class

        # ----- !! BLOCKING WHILE LOOP !! ----- #
        while not self.cli_lift_zero.wait_for_service(timeout_sec=1):
            self.get_logger().warn("Waiting for the lift/zero service to be available (BLOCKING)")
        self.cli_lift_zero.call_async(Stop.Request())  # Zero the lift by slowly raising it up

    # NOTE: This method is meant to find a safe digging location on the field, but it has not been tested enough yet.
    # def optimal_dig_location(self) -> list:
    #     available_dig_spots = []
    #     try:
    #         costmap = PyCostmap2D(self.nav2.getGlobalCostmap())
    #         resolution = costmap.getResolution()
    #         print(resolution)
    #         # NEEDED MEASUREMENTS:
    #         robot_width = 1.749 / 2
    #         robot_width_pixels = robot_width // resolution
    #         danger_threshold, real_danger_threshold = 50, 150
    #         dig_zone_depth, dig_zone_start, dig_zone_end = 2.57, 4.07, 8.14
    #         dig_zone_border_y = 2.0
    #         while len(available_dig_spots) == 0:
    #             if danger_threshold > real_danger_threshold:
    #                 self.get_logger().warn("No safe digging spots available. Switch to manual control.")
    #                 return None
    #             i = dig_zone_start + robot_width
    #             while i <= dig_zone_end - robot_width:
    #                 if (
    #                     costmap.getDigCost(i, dig_zone_border_y, robot_width_pixels, dig_zone_depth)
    #                     <= self.DANGER_THRESHOLD
    #                 ):
    #                     available_dig_spots.append(create_pose_stamped(i, -dig_zone_border_y, 270))
    #                     i += robot_width
    #                 else:
    #                     i += resolution
    #             if len(available_dig_spots) > 0:
    #                 return available_dig_spots
    #             danger_threshold += 5
    #     except Exception as e:
    #         self.get_logger().error(f"Error in optimal_dig_location: {e} on line {sys.exc_info()[-1].tb_lineno}")
    #         return None

    def stop_all_subsystems(self) -> None:
        """This method stops all subsystems on the robot."""
        self.cli_skimmer_stop.call_async(Stop.Request())  # Stop the skimmer belt
        self.cli_drivetrain_stop.call_async(Stop.Request())  # Stop the drivetrain
        self.cli_lift_stop.call_async(Stop.Request())  # Stop the skimmer lift

    def end_autonomous(self) -> None:
        """This method returns to teleop control."""
        self.stop_all_subsystems()  # Stop all subsystems
        self.state = states["Teleop"]  # Return to Teleop mode

    # TODO: This should not be needed anymore after ticket #257 is implemented!
    def skimmer_goal_callback(self, msg: Bool) -> None:
        """Update the member variable accordingly."""
        self.skimmer_goal_reached = msg.data

    def get_result_callback(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Autonomous Goal succeeded!")
            self.end_autonomous()

    def calibrate_goal_response_callback(self, future: Future):
        self.field_calibrated_handle: ClientGoalHandle = future.result()
        if not self.field_calibrated_handle.accepted:
            self.get_logger().info("Field calibration Goal rejected")
            return
        field_calibrated: Future = self.field_calibrated_handle.get_result_async()
        field_calibrated.add_done_callback(self.get_result_callback)

    def auto_offload_goal_response_callback(self, future: Future):
        self.auto_offload_handle: ClientGoalHandle = future.result()
        if not self.auto_offload_handle.accepted:
            self.get_logger().info("Auto offload Goal rejected")
            return
        result: Future = self.auto_offload_handle.get_result_async()
        result.add_done_callback(self.get_result_callback)

    def auto_dig_goal_response_callback(self, future: Future):
        self.auto_dig_handle: ClientGoalHandle = future.result()
        if not self.auto_dig_handle.accepted:
            self.get_logger().info("Auto dig Goal rejected")
            return
        result: Future = self.auto_dig_handle.get_result_async()
        result.add_done_callback(self.get_result_callback)

    def joystick_callback(self, msg: Joy) -> None:
        """This method is called whenever a joystick message is received."""

        # PUT TELEOP CONTROLS BELOW #

        if self.state == states["Teleop"]:
            # Drive the robot using joystick input during Teleop
            forward_power = msg.axes[bindings.RIGHT_JOYSTICK_VERTICAL_AXIS] * self.max_drive_power  # Forward power
            horizontal_power = (
                msg.axes[bindings.RIGHT_JOYSTICK_HORIZONTAL_AXIS] * self.max_drive_power
            )  # Horizontal power
            turn_power = msg.axes[bindings.LEFT_JOYSTICK_HORIZONTAL_AXIS] * self.max_turn_power  # Turning power
            self.drive_power_publisher.publish(
                Twist(
                    linear=Vector3(x=forward_power, y=horizontal_power),
                    angular=Vector3(z=turn_power),
                )
            )

            # Check if the skimmer button is pressed #
            if msg.buttons[bindings.X_BUTTON] == 1 and buttons[bindings.X_BUTTON] == 0:
                self.cli_skimmer_toggle.call_async(SetPower.Request(power=self.skimmer_belt_power))

            # Check if the reverse skimmer button is pressed #
            if msg.buttons[bindings.Y_BUTTON] == 1 and buttons[bindings.Y_BUTTON] == 0:
                self.cli_skimmer_setPower.call_async(SetPower.Request(power=-self.skimmer_belt_power))

            # Check if the lift dumping position button is pressed #
            if msg.buttons[bindings.B_BUTTON] == 1 and buttons[bindings.B_BUTTON] == 0:
                self.cli_lift_setPosition.call_async(SetPosition.Request(position=self.lift_dumping_position))

            # Check if the lift digging position button is pressed #
            if msg.buttons[bindings.A_BUTTON] == 1 and buttons[bindings.A_BUTTON] == 0:
                self.cli_lift_setPosition.call_async(SetPosition.Request(position=self.lift_digging_start_position))

            # Manually adjust the height of the skimmer with the left and right triggers
            if msg.buttons[bindings.RIGHT_TRIGGER] == 1 and buttons[bindings.RIGHT_TRIGGER] == 0:
                self.cli_lift_set_power.call_async(SetPower.Request(power=self.skimmer_lift_manual_power))
            elif msg.buttons[bindings.RIGHT_TRIGGER] == 0 and buttons[bindings.RIGHT_TRIGGER] == 1:
                self.cli_lift_stop.call_async(Stop.Request())
            elif msg.buttons[bindings.LEFT_TRIGGER] == 1 and buttons[bindings.LEFT_TRIGGER] == 0:
                self.cli_lift_set_power.call_async(SetPower.Request(power=-self.skimmer_lift_manual_power))
            elif msg.buttons[bindings.LEFT_TRIGGER] == 0 and buttons[bindings.LEFT_TRIGGER] == 1:
                self.cli_lift_stop.call_async(Stop.Request())

            # Check if the calibration button is pressed
            if msg.buttons[bindings.RIGHT_BUMPER] == 1 and buttons[bindings.RIGHT_BUMPER] == 0:
                self.cli_drivetrain_calibrate.call_async(Stop.Request())

        # THE CONTROLS BELOW ALWAYS WORK #

        # Check if the Apriltag calibration button is pressed
        # TODO: This needs to be tested on the physical robot!
        if msg.buttons[bindings.START_BUTTON] == 1 and buttons[bindings.START_BUTTON] == 0:
            # Check if the field calibration process is not running
            if self.field_calibrated_handle.status != GoalStatus.STATUS_EXECUTING:
                if not self.act_calibrate_field_coordinates.wait_for_server(timeout_sec=1.0):
                    self.get_logger().error("Field calibration action not available")
                    return
                goal = CalibrateFieldCoordinates.Goal()
                field_calibrated_request = self.act_calibrate_field_coordinates.send_goal_async(goal)
                field_calibrated_request.add_done_callback(self.calibrate_goal_response_callback)
                self.stop_all_subsystems()  # Stop all subsystems
                self.state = states["Autonomous"]  # Exit Teleop mode
            # Terminate the field calibration process
            else:
                self.get_logger().warn("Field Calibration Terminated")
                self.field_calibrated_handle.cancel_goal_async()
                self.end_autonomous()  # Return to Teleop mode

        # Check if the autonomous digging button is pressed
        # TODO: This needs to be tested extensively on the physical robot!
        if msg.buttons[bindings.BACK_BUTTON] == 1 and buttons[bindings.BACK_BUTTON] == 0:
            # Check if the auto digging process is not running
            if self.auto_dig_handle.status != GoalStatus.STATUS_EXECUTING:
                if not self.act_auto_dig.wait_for_server(timeout_sec=1.0):
                    self.get_logger().error("Auto dig action not available")
                    return
                goal = AutoDig.Goal(
                    lift_dumping_position=self.lift_dumping_position,
                    lift_digging_start_position=self.lift_digging_start_position,
                    skimmer_belt_power=self.skimmer_belt_power,
                )
                auto_dig_request = self.act_auto_dig.send_goal_async(goal)
                auto_dig_request.add_done_callback(self.auto_dig_goal_response_callback)
                self.stop_all_subsystems()  # Stop all subsystems
                self.state = states["Autonomous"]  # Exit Teleop mode
            # Terminate the auto dig process
            else:
                self.get_logger().warn("Auto Dig Terminated")
                self.auto_dig_handle.cancel_goal_async()
                self.end_autonomous()  # Return to Teleop mode

        # Check if the autonomous offload button is pressed
        # TODO: This needs to be tested extensively on the physical robot!
        if msg.buttons[bindings.LEFT_BUMPER] == 1 and buttons[bindings.LEFT_BUMPER] == 0:
            # Check if the auto offload process is not running
            if self.auto_offload_handle.status != GoalStatus.STATUS_EXECUTING:
                if not self.act_auto_offload.wait_for_server(timeout_sec=1.0):
                    self.get_logger().error("Auto offload action not available")
                    return
                goal = AutoOffload.Goal(
                    lift_dumping_position=self.lift_dumping_position,
                    skimmer_belt_power=self.skimmer_belt_power,
                )
                auto_offload_request = self.act_auto_offload.send_goal_async(goal)
                auto_offload_request.add_done_callback(self.auto_offload_goal_response_callback)
                self.stop_all_subsystems()  # Stop all subsystems
                self.state = states["Autonomous"]  # Exit Teleop mode
            # Terminate the auto offload process
            else:
                self.get_logger().warn("Auto Offload Terminated")
                self.auto_offload_handle.cancel_goal_async()
                self.end_autonomous()  # Return to Teleop mode

        # Update button states (this allows us to detect changing button states)
        for index in range(len(buttons)):
            buttons[index] = msg.buttons[index]


async def spin(executor: MultiThreadedExecutor) -> None:
    """This function is called in the main function to run the executor."""
    while rclpy.ok():  # While ROS is still running
        executor.spin_once()  # Spin the executor once
        await asyncio.sleep(0)  # Setting the delay to 0 provides an optimized path to allow other tasks to run.


def main(args=None) -> None:
    rclpy.init(args=args)

    main_node = MainControlNode()  # Instantiate the node
    executor = MultiThreadedExecutor()  # Create an executor
    executor.add_node(main_node)  # Add the node to the executor
    main_node.get_logger().info("Hello from the rovr_control package!")

    loop = asyncio.get_event_loop()  # Get the event loop
    loop.run_until_complete(spin(executor))  # Run the spin function in the event loop

    # Clean up and shutdown
    main_node.nav2.lifecycleShutdown()
    main_node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
