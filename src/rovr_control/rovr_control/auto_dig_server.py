import rclpy
from rclpy.action import ActionServer


from rovr_interfaces.action import AutoDig
from rovr_interfaces.srv import SetPosition, SetPower
from rclpy.action.server import ServerGoalHandle, CancelResponse, GoalResponse
from std_srvs.srv import Trigger, SetBool

from rovr_control.node_util import AsyncNode


class AutoDigServer(AsyncNode):
    def __init__(self):
        super().__init__("auto_dig_server")
        self._action_server = ActionServer(
            self, AutoDig, "auto_dig", self.execute_callback, goal_callback=self.goal_callback, handle_accepted_callback=self.handle_accepted_callback, cancel_callback=self.cancel_callback
        )

        self.cli_lift_zero = self.create_client(Trigger, "lift/zero")
        self.cli_lift_setPosition = self.create_client(SetPosition, "lift/setPosition")
        self.cli_lift_set_power = self.create_client(SetPower, "lift/setPower")
        self.cli_lift_stop = self.create_client(Trigger, "lift/stop")
        self.cli_digger_stop = self.create_client(Trigger, "digger/stop")
        self.cli_digger_setPower = self.create_client(SetPower, "digger/setPower")
        self.cli_big_agitator_on_off = self.create_client(SetBool, "big_agitator_on_off")
        self.cli_small_agitator_on_off = self.create_client(SetBool, "small_agitator_on_off")

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
        if not self.cli_big_agitator_on_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Agitator motor on/off service not available")
            goal_handle.abort()
            return result
        if not self.cli_small_agitator_on_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Agitator motor on/off service not available")
            goal_handle.abort()
            return result

        # Lower the digger so that it is just above the ground (get to this position fast)
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Lowering the digger to the starting position")
            await self.cli_lift_setPosition.call_async(
                SetPosition.Request(position=goal_handle.request.lift_digging_start_position)
            )
        fails = 0
        max_fails = 4
        self.goal_handle = goal_handle

        fails += await self.set_position_retry(300.0, 0.12, max_fails-fails)

        fails += await self.set_position_retry(400.0, 0.118, max_fails-fails)

        # if not goal_handle.is_cancel_requested and fails < max_fails:
        #     # Start the agitator motor
        #     self.get_logger().info("Starting Small Agitator Motor")
        #     await self.cli_small_agitator_on_off.call_async(SetBool.Request(data=True))


        fails += await self.set_position_retry(475.0, 0.108, max_fails-fails)

        fails += await self.set_position_retry(525.0, 0.098, max_fails-fails)

        if not goal_handle.is_cancel_requested and fails < max_fails:
            # Start the agitator motor
            self.get_logger().info("Starting BIG Agitator Motor")
            await self.cli_big_agitator_on_off.call_async(SetBool.Request(data=True))

        fails += await self.set_position_retry(575.0, 0.098, max_fails-fails)

        fails += await self.set_position_retry(650.0, 0.088, max_fails - fails)

        fails += await self.set_position_retry(725.0, 0.098, max_fails - fails)

        # fails += await self.set_position_retry(750.0, 0.105, max_fails-fails)

        fails += await self.set_position_retry(800.0, 0.103, max_fails - fails)

        # fails += await self.set_position_retry(850.0, 0.11, max_fails-fails)

        fails += await self.set_position_retry(900.0, 0.108, max_fails - fails)

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
            await self.cli_small_agitator_on_off.call_async(SetBool.Request(data=False))

        if not goal_handle.is_cancel_requested:
            # Stop the agitator motor
            self.get_logger().info("Stopping Agitator Motor")
            await self.cli_big_agitator_on_off.call_async(SetBool.Request(data=False))

        # Raise the digger so that it is just below the safety zone
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Raising the digger to the starting position")
            await self.cli_lift_setPosition.call_async(
                SetPosition.Request(position=goal_handle.request.lift_digging_start_position)
            )

        # Start the digger chain
        if not goal_handle.is_cancel_requested:
            self.get_logger().info("Starting the digger chain")
            await self.cli_digger_setPower.call_async(SetPower.Request(power=goal_handle.request.digger_chain_power))
            await self.async_sleep(7)

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

    async def cancel_callback(self, cancel_request: ServerGoalHandle):
        """This method is called when the action is canceled."""
        super().cancel_callback(cancel_request)
        self.get_logger().info("Goal is cancelling")
        self.cli_digger_stop.call_async(Trigger.Request())
        self.cli_big_agitator_on_off.call_async(SetBool.Request(data=False))
        self.cli_small_agitator_on_off.call_async(SetBool.Request(data=False))
        self.cli_lift_stop.call_async(Trigger.Request())
        return CancelResponse.ACCEPT

    async def set_position_retry(self, position: float, power_limit: float, max_retries: int = 4):
        if not self.goal_handle.is_cancel_requested:
            self.get_logger().info("Starting the digger chain")
            await self.cli_digger_setPower.call_async(SetPower.Request(power=self.goal_handle.request.digger_chain_power))

            for i in range(max_retries):
                if not self.goal_handle.is_cancel_requested:
                    self.get_logger().info(f"Attempting to set position to {position} with power limit {power_limit}")
                    if (await self.cli_lift_setPosition.call_async(SetPosition.Request(position=position, power_limit=power_limit))).success:
                        self.get_logger().info(f"Successfully set position to {position}")
                        return i

                    self.get_logger().info(f"Failed to set position to {position}")
                    
                    if i == max_retries - 1:
                        break

                    if self.goal_handle.is_cancel_requested:
                        self.cli_lift_stop.call_async(Trigger.Request())
                        return -1

                    await self.async_sleep(1)
                    await self.cli_digger_setPower.call_async(SetPower.Request(power=self.goal_handle.request.digger_chain_power))
                    await self.async_sleep(5)

                else:
                    return -1

            return max_retries
        return -1

def main(args=None) -> None:
    rclpy.init(args=args)

    action_server = AutoDigServer()
    rclpy.spin(action_server)

    action_server.destroy_node()
    rclpy.shutdown()

# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
