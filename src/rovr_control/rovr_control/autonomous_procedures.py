import asyncio  # Allows the creation of asynchronous autonomous procedures!
from rclpy.node import Node  # Only used for type hints
from typing import Callable  # Only used for type hints

# Import custom ROS 2 interfaces
from rovr_interfaces.srv import ConveyorSetPower, SetPower, ReadSerial
from rovr_interfaces.srv import Stop, Drive, LinearActuator


class autonomous_procedure:
    """This class is a template used to define an autonomous procedure."""

    def __init__(self, rovr_control_instance: Node, name: str, procedure_logic: Callable) -> None:
        self.name = name
        self.node_instance = rovr_control_instance
        self.procedure_logic = procedure_logic

    async def run(self):
        self.node_instance.stop_all_subsystems()  # Stop all subsystems
        self.node_instance.state = self.node_instance.states[self.name]
        print(f"\nStarting {self.name} Procedure!")
        try:
            await self.procedure_logic(self.node_instance)
            print(f"{self.name} Procedure Complete!\n")
            self.node_instance.stop_all_subsystems()  # Stop all subsystems
            self.node_instance.state = self.node_instance.states["Teleop"]  # Return to Teleop control
        except asyncio.CancelledError:
            self.terminate()  # If the procedure is cancelled, terminate it
        except Exception as e:
            print(f"Error in {self.name} Procedure: {e}")
            self.terminate()  # If the procedure encounters an error, terminate it

    def terminate(self):
        print(f"{self.name} Procedure Terminated\n")
        self.node_instance.stop_all_subsystems()  # Stop all subsystems
        self.node_instance.state = self.node_instance.states["Teleop"]  # Return to Teleop control


# Define all autonomous procedures below! #


async def auto_dig_procedure(node_instance: Node) -> None:
    """This method lays out the procedure for autonomously digging!"""
    await node_instance.cli_digger_setPower.call_async(SetPower.Request(power=node_instance.digger_rotation_power))
    await node_instance.cli_conveyor_setPower.call_async(
        ConveyorSetPower.Request(
            drum_belt_power=node_instance.drum_belt_power, conveyor_belt_power=node_instance.conveyor_belt_power
        )
    )
    # Read all messages from the Arduino serial buffer to clear them out
    await node_instance.cli_digger_read_all.call_async(ReadSerial.Request())
    await asyncio.sleep(2)  # Wait a bit for the drum motor to get up to speed
    # Extend the linear actuator
    await node_instance.cli_digger_extend.call_async(
        LinearActuator.Request(extend_power=node_instance.linear_actuator_power)
    )
    # Wait for a confirmation message from the Arduino
    reading = await node_instance.cli_digger_read.call_async(ReadSerial.Request())
    while reading.data != "f":  # this is just the character we arbitrarily chose in the Arduino code:
        reading = await node_instance.cli_digger_read.call_async(ReadSerial.Request())
        await asyncio.sleep(0)  # trick to allow other tasks to run ;)
    await asyncio.sleep(5)  # Wait for 5 seconds
    # Retract the linear actuator
    await node_instance.cli_digger_retract.call_async(
        LinearActuator.Request(retract_power=node_instance.linear_actuator_up_power)
    )
    # Wait for a confirmation message from the Arduino
    reading = await node_instance.cli_digger_read.call_async(ReadSerial.Request())
    while reading.data != "s":  # this is just the character we arbitrarily chose in the Arduino code
        reading = await node_instance.cli_digger_read.call_async(ReadSerial.Request())
        await asyncio.sleep(0)  # trick to allow other tasks to run ;)
    await node_instance.cli_digger_stop.call_async(Stop.Request())
    await asyncio.sleep(0.5)  # Let the digger slow down
    # Reverse the digging drum
    await node_instance.cli_digger_setPower.call_async(SetPower.Request(power=-1 * node_instance.digger_rotation_power))
    await asyncio.sleep(5)  # Wait for 5 seconds
    await node_instance.cli_digger_stop.call_async(Stop.Request())
    await node_instance.cli_conveyor_stop.call_async(Stop.Request())


async def auto_offload_procedure(node_instance: Node):
    """This method lays out the procedure for autonomously offloading!"""
    print("Searching for an Apriltag to dock with...")  # Search for an Apriltag before continuing
    node_instance.apriltagX = 0.0
    await asyncio.sleep(0.1)  # Add a small delay in case we can see an Apriltag already
    if node_instance.apriltagX == 0.0:
        # Start turning slowly to look for an Apriltag
        await node_instance.cli_drivetrain_drive.call_async(Drive.Request(forward_power=0.0, turning_power=0.3))
        while node_instance.apriltagX == 0.0:
            print("searching")
            await asyncio.sleep(0)  # Trick to allow other tasks to run ;)
        print(
            f"Apriltag found! x: {node_instance.apriltagX}, z: {node_instance.apriltagZ}, yaw :{node_instance.apriltagYaw}"
        )
        await node_instance.cli_drivetrain_stop.call_async(Stop.Request())  # Stop turning
    # Continue correcting until we are within 1.5 meters of the tag # TODO: Tune this distance
    while node_instance.apriltagZ >= 1.5:
        # TODO: Tune both of these P constants on the actual robot
        turn = -1 * (0.5 * node_instance.apriltagYaw + 0.5 * node_instance.apriltagX)
        await node_instance.cli_drivetrain_drive.call_async(
            Drive.Request(forward_power=node_instance.autonomous_driving_power, turning_power=turn)
        )
        print(
            f"Tracking Apriltag with pose x: {node_instance.apriltagX}, z: {node_instance.apriltagZ}, yaw :{node_instance.apriltagYaw}"
        )
        await asyncio.sleep(0.1)  # Add a small delay so we don't overload ROS with too many messages
    await node_instance.cli_drivetrain_stop.call_async(Stop.Request())  # Stop the robot
    print("Docking with the trough")  # Finish docking with the trough
    await node_instance.cli_drivetrain_drive.call_async(
        Drive.Request(forward_power=node_instance.autonomous_driving_power, turning_power=0.0)
    )
    await asyncio.sleep(4)  # TODO: Tune this timing (how long to drive straight for at the end of docking)
    await node_instance.cli_drivetrain_stop.call_async(Stop.Request())
    print("Commence Offloading!")
    await node_instance.cli_offloader_setPower.call_async(SetPower.Request(power=node_instance.offload_belt_power))
    await asyncio.sleep(10)  # TODO: Tune this timing (how long to run the offloader for)
    await node_instance.cli_offloader_stop.call_async(Stop.Request())  # stop offloading
