import rclpy
from rclpy.action import ActionServer

from rovr_interfaces.action import AutoDig
from rovr_interfaces.srv import SetPosition, SetPower
from rclpy.action.server import ServerGoalHandle, CancelResponse
from std_srvs.srv import Trigger, SetBool

from rovr_control.node_util import AsyncNode

class AutoDigServer(AsyncNode):
    def __init__(self):
        super().__init__("auto_dig_server")
        self._action_server = ActionServer(
            self, AutoDig, "auto_dig", self.execute_callback, cancel_callback=self.cancel_callback
        )

        # tilt
        self.set_tilt = self.create_client(Trigger, "lift/zero") # /actuator_tilt/setExtension
        self.stop_tilt = self.create_client(SetPosition, "lift/setPosition") # /actuator_tilt/stop

        # extend
        self.set_extension = self.create_client(SetPower, "lift/setPower") # /motor_push/setPosition
        self.stop_extension = self.create_client(Trigger, "lift/stop") # /motor_push/stop
        self.retract_extender = self.create_client(Trigger, "lift/retractExtender") # /actuator_extender/retractExtender

        # spin auger
        self.auger_stop = self.create_client(Trigger, "digger/stop") # /motor_spin/stop
        self.auger_start = self.create_client(Trigger, "digger/setPower") # /motor_spin/run 

        # agitator
        self.cli_motor_on_off = self.create_client(SetBool, "motor_on_off")
        self.cli_motor_toggle = self.create_client(Trigger, "motor_toggle")

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Starting Autonomous Digging Procedure!")
        result = AutoDig.Result()

        # Make sure the services are available
        if not self.cli_lift_setPosition.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Lift set position service not available")
            goal_handle.abort()
            return result
        if not self.cli_lift_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Lift stop service not available")
            goal_handle.abort()
            return result
        if not self.cli_lift_set_power.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Lift set power service not available")
            goal_handle.abort()
            return result
        if not self.cli_lift_zero.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Lift zero service not available")
            goal_handle.abort()
            return result
        if not self.cli_digger_setPower.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Digger set power service not available")
            goal_handle.abort()
            return result
        if not self.cli_digger_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Digger stop service not available")
            goal_handle.abort()
            return result
        if not self.cli_motor_on_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Agitator motor on/off service not available")
            goal_handle.abort()
            return result
        if not self.cli_motor_toggle.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Agitator motor toggle service not available")
            goal_handle.abort()
            return result
        
        # Tilt the auger into digging position
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Tilting the auger into digging position")
            await self.set_tilt.call_async(
                position=goal_handle.request.tilt_digging_start_position
            )
            await self.auger_start.call_async(Trigger.request())

        fails = 0
        max_fails = 4
        self.goal_handle = goal_handle

        fails += await self.set_position_retry(400.0, 0.12, max_fails - fails)

        fails += await self.set_position_retry(475.0, 0.11, max_fails - fails)

        fails += await self.set_position_retry(550.0, 0.10, max_fails - fails)

        if not goal_handle.is_cancel_requested and fails < max_fails:
            # Start the agitator motor
            self.get_logger().info("Starting Agitator Motor")
            await self.cli_motor_on_off.call_async(SetBool.Request(data=True))

        fails += await self.set_position_retry(650.0, 0.09, max_fails - fails)

        fails += await self.set_position_retry(725.0, 0.10, max_fails - fails)

        # fails += await self.set_position_retry(750.0, 0.105, max_fails-fails)

        fails += await self.set_position_retry(800.0, 0.105, max_fails - fails)

        # fails += await self.set_position_retry(850.0, 0.11, max_fails-fails)

        fails += await self.set_position_retry(900.0, 0.11, max_fails - fails)

        # Dig in place (no lift lowering) for 5 seconds
        if not goal_handle.is_cancel_requested:
            await self.cli_digger_setPower.call_async(SetPower.Request(power=goal_handle.request.digger_chain_power))
            self.get_logger().info("Auto Digging in Place")
            await self.async_sleep(5)
            self.get_logger().info("Done Digging in Place")
            self.cli_digger_stop.call_async(Trigger.Request())

        if not goal_handle.is_cancel_requested:
            # Stop the agitator motor
            self.get_logger().info("Stopping Agitator Motor")
            await self.cli_motor_on_off.call_async(SetBool.Request(data=False))

        # Raise the digger so that it is just below the safety zone
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Raising the digger to the starting position")
            await self.cli_lift_setPosition.call_async(
                SetPosition.Request(position=goal_handle.request.tilt_digging_start_position)
            )

        # Start the digger chain
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Starting the digger chain")
            await self.cli_digger_setPower.call_async(SetPower.Request(power=goal_handle.request.digger_chain_power))
            await self.async_sleep(5)

        # Raise the digger back up to the top using the lift
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Raising the digger up to the top")
            await self.cli_lift_zero.call_async(Trigger.Request())

        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Autonomous Digging Procedure Complete!")
            goal_handle.succeed()
            return result
        else:
            self.get_logger().info("Goal was cancelled")
            goal_handle.canceled()
            return result

    def cancel_callback(self, cancel_request: ServerGoalHandle):
        """This method is called when the action is canceled."""
        super().cancel_callback(cancel_request)
        self.get_logger().info("Goal is cancelling")
        self.cli_digger_stop.call_async(Trigger.Request())
        self.cli_lift_stop.call_async(Trigger.Request())
        self.cli_motor_on_off.call_async(SetBool.Request(data=False))
        return CancelResponse.ACCEPT

    async def set_position_retry(self, position: float, power_limit: float, max_retries: int = 4):
        self.get_logger().info("Starting the digger chain")
        await self.cli_digger_setPower.call_async(SetPower.Request(power=self.goal_handle.request.digger_chain_power))

        for i in range(max_retries):
            if not self.goal_handle.is_cancel_requested:
                self.get_logger().info(f"Attempting to set position to {position} with power limit {power_limit}")
                if (
                    await self.cli_lift_setPosition.call_async(
                        SetPosition.Request(position=position, power_limit=power_limit)
                    )
                ).success:
                    self.get_logger().info(f"Successfully set position to {position}")
                    return i

                if i == max_retries - 1:
                    break

                await self.async_sleep(1)
                await self.cli_digger_setPower.call_async(
                    SetPower.Request(power=self.goal_handle.request.digger_chain_power)
                )
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
