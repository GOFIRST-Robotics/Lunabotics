import rclpy
from rclpy.action import ActionServer

from rovr_interfaces.action import AutoDig
from rovr_interfaces.srv import SetExtension, SetPower
from rovr_interfaces.srv import AugerSetPushMotor
from rclpy.action.server import ServerGoalHandle, CancelResponse
from std_srvs.srv import Trigger, SetBool

from rovr_control.node_util import AsyncNode

class AutoDigServer(AsyncNode):
    def __init__(self):
        super().__init__("auto_dig_server")
        self._action_server = ActionServer(
            self, AutoDig, "auto_dig", self.execute_callback,
            goal_callback=self.goal_callback, handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )

        # tilt
        self.set_tilt = self.create_client(SetExtension, "auger/tilt_actuator/setExtension") # /actuator_tilt/setExtension
        self.stop_tilt = self.create_client(Trigger, "auger/tilt_actuator/stop") # /actuator_tilt/stop

        # extend
        self.set_extension = self.create_client(AugerSetPushMotor, "auger/push_motor/setPosition")
        self.stop_extension = self.create_client(Trigger, "auger/push_motor/stop")
        self.retract_extender = self.create_client(Trigger, "auger/push_motor/retract")

        # spin auger
        self.screw_stop = self.create_client(Trigger, "auger/screw/stop") # /motor_spin/stop
        self.screw_start = self.create_client(Trigger, "auger/screw/run") # /motor_spin/run 

        # agitator #TODO: uncomment if needed.
        # self.agitator = self.create_client(SetBool, "motor_on_off")
        # self.cli_motor_toggle = self.create_client(Trigger, "motor_toggle")

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info('Starting new goal')
        goal_handle.execute()

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Starting Autonomous Digging Procedure!")
        result = AutoDig.Result()

        # Make sure the services are available
        if not self.set_tilt.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Tilt set position service not available")
            goal_handle.abort()
            return result
        if not self.stop_tilt.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Tilt stop service not available")
            goal_handle.abort()
            return result
        if not self.set_extension.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Extension set power service not available")
            goal_handle.abort()
            return result
        if not self.retract_extender.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Extension retract service not available")
            goal_handle.abort()
            return result
        if not self.stop_extension.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Extension stop service not available")
            goal_handle.abort()
            return result
        if not self.screw_start.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Auger set power service not available")
            goal_handle.abort()
            return result
        if not self.screw_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Auger stop service not available")
            goal_handle.abort()
            return result
        # if not self.agitator.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().error("Agitator motor on/off service not available")
        #     goal_handle.abort()
        #     return result
        # if not self.cli_motor_toggle.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().error("Agitator motor toggle service not available")
        #     goal_handle.abort()
        #     return result
        
        # Tilt the auger into digging position
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Tilting the auger into digging position")
            await self.set_tilt.call_async(SetExtension.Request(extension=True))
            await self.screw_start.call_async(Trigger.Request())

        fails = 0
        max_fails = 4
        self.goal_handle = goal_handle

        #TODO: All of these numbers need to be tuned based on the actual robot and digging conditions.
        fails += await self.set_position_retry(400.0, 0.12, max_fails - fails)

        fails += await self.set_position_retry(475.0, 0.108, max_fails-fails)

        fails += await self.set_position_retry(525.0, 0.098, max_fails-fails)

        # if not goal_handle.is_cancel_requested and fails < max_fails:
        #    # Start the agitator motor
        #    self.get_logger().info("Starting Agitator Motor")
        #    await self.agitator.call_async(SetBool.Request(data=True))

        fails += await self.set_position_retry(575.0, 0.098, max_fails-fails)

        fails += await self.set_position_retry(650.0, 0.088, max_fails - fails)

        fails += await self.set_position_retry(725.0, 0.098, max_fails - fails)

        # fails += await self.set_position_retry(750.0, 0.105, max_fails-fails)

        fails += await self.set_position_retry(800.0, 0.103, max_fails - fails)

        # fails += await self.set_position_retry(850.0, 0.11, max_fails-fails)

        fails += await self.set_position_retry(900.0, 0.108, max_fails - fails)

        # Dig in place (no lift lowering) for 5 seconds
        if not goal_handle.is_cancel_requested:
            await self.screw_start.call_async(Trigger.Request())
            self.get_logger().info("Auto Digging in Place")
            await self.async_sleep(5)
            self.get_logger().info("Done Digging in Place")
        
        # TODO: uncomment if using auger.
        # if not goal_handle.is_cancel_requested:
        #     # Stop the agitator motor
        #     self.get_logger().info("Stopping Agitator Motor")
        #     await self.agitator.call_async(SetBool.Request(data=False))

        # TODO: Uncomment if we decide to spin while raising.
        # Raise the digger so that it is just below the safety zone
        # if not goal_handle.is_cancel_requested:
        #     self.get_logger().info("Raising the digger to the starting position")
        #     await self.cli_lift_setPosition.call_async(
        #         SetPosition.Request(position=goal_handle.request.tilt_digging_start_position)
        #     )

        # Start the digger chain
        # if not goal_handle.is_cancel_requested:
        #     self.get_logger().info("Starting the digger chain")
        #     await self.screw_start.call_async(SetPower.Request(power=goal_handle.request.digger_chain_power))
        #     await self.async_sleep(5)

        # Raise the digger back up to the top using the lift
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Raising the digger up to the top")
            await self.retract_extender.call_async(Trigger.Request())

        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Stopping the digger chain")
            await self.screw_stop.call_async(Trigger.Request())

        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Resetting tilt")
            await self.set_tilt.call_async(SetExtension.Request(extension=False))

        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Autonomous Digging Procedure Complete!")
            goal_handle.succeed()
            return result
        else:
            self.get_logger().info("Goal was cancelled")
            goal_handle.canceled()
            return result

    async def cancel_callback(self, cancel_request: ServerGoalHandle):
        """This method is called when the action is canceled."""
        super().cancel_callback(cancel_request)
        self.get_logger().info("Goal is cancelling")
        self.screw_stop.call_async(Trigger.Request())
        self.cli_lift_stop.call_async(Trigger.Request())
        self.agitator.call_async(SetBool.Request(data=False))
        return CancelResponse.ACCEPT

    async def set_position_retry(self, position: float, power_limit: float, max_retries: int = 4):
        self.get_logger().info("Starting the digger chain")
        await self.screw_start.call_async(SetPower.Request(power=self.goal_handle.request.digger_chain_power))

        for i in range(max_retries):
            if not self.goal_handle.is_cancel_requested:
                self.get_logger().info(f"Attempting to set position to {position} with power limit {power_limit}")
                if (
                    await self.cli_lift_setPosition.call_async(
                        AugerSetPushMotor.Request(position=position, speed=power_limit)
                    )
                ).success:
                    self.get_logger().info(f"Successfully set position to {position}")
                    return i

                if i == max_retries - 1:
                    break

                await self.async_sleep(1)
                await self.screw_start.call_async(Trigger.Request())
                await self.async_sleep(5)

            else:
                return -1

        return max_retries


def main(args=None) -> None:
    rclpy.init(args=args)

    action_server = AutoDigServer()
    rclpy.spin(action_server)

    action_server.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
