# This ROS 2 node contains code for the conveyor subsystem of the robot.
# Original Author: Anthony Brogni <brogn002@umn.edu> in Fall 2023
# Maintainer: Anthony Brogni <brogn002@umn.edu>
# Last Updated: September 2023

# Import the ROS 2 Python module
import rclpy
from rclpy.node import Node

# Import custom ROS 2 interfaces
from rovr_interfaces.msg import MotorCommand
from rovr_interfaces.srv import ConveyorToggle
from rovr_interfaces.srv import ConveyorStop
from rovr_interfaces.srv import ConveyorSetPower

# Define helper functions here
def clamp(number: float, minimum: float, maximum: float) -> float:
    """Clamps a number between the specified minimum and maximum."""
    return max(minimum, min(number, maximum))


class ConveyorNode(Node):
    def __init__(self):
        """Initialize the ROS 2 conveyor node."""
        super().__init__("conveyor")
        
        # Define publishers and subscribers here
        self.motor_command_pub = self.create_publisher(MotorCommand, "motor_cmd", 10)
        
        # Define services (methods callable from the outside) here
        self.srv_toggle = self.create_service(ConveyorToggle, 'conveyor/toggle', self.toggle_callback)
        self.srv_stop = self.create_service(ConveyorStop, 'conveyor/stop', self.stop_callback)
        self.srv_setPower = self.create_service(ConveyorSetPower, 'conveyor/setPower', self.set_power_callback)
        
        # Define motor CAN IDs here
        self.DRUM_BELT_MOTOR = 7
        self.CONVEYOR_BELT_MOTOR = 6
        
        # Current subsystem state
        self.running = False

    # Define subsystem methods here
    def set_power(self, drum_belt_power: float, conveyor_belt_power: float) -> None:
        """This method sets power to the conveyor belts."""
        self.running = True
        drum_belt_power = clamp(drum_belt_power, -1.0, 1.0)  # Clamp the power between -1.0 and 1.0
        conveyor_power = clamp(conveyor_belt_power, -1.0, 1.0)  # Clamp the power between -1.0 and 1.0
        self.motor_command_pub.publish(MotorCommand(can_id=self.DRUM_BELT_MOTOR, type="duty_cycle", value=drum_belt_power))
        self.motor_command_pub.publish(MotorCommand(can_id=self.CONVEYOR_BELT_MOTOR, type="duty_cycle", value=conveyor_power))

    def stop(self) -> None:
        """This method stops both conveyor belts."""
        self.running = False
        self.motor_command_pub.publish(MotorCommand(can_id=self.DRUM_BELT_MOTOR, type="duty_cycle", value=0.0))
        self.motor_command_pub.publish(MotorCommand(can_id=self.CONVEYOR_BELT_MOTOR, type="duty_cycle", value=0.0))

    def toggle(self, drum_belt_power: float, conveyor_belt_power: float) -> None:
        """This method toggles the conveyor belts."""
        if self.running:
            self.stop()
        else:
            self.set_power(drum_belt_power, conveyor_belt_power)
            
    # Define service callback methods here
    def set_power_callback(self, request, response) -> None:
        """This service request sets power to the offloading belt."""
        self.set_power(request.drum_belt_power, request.conveyor_belt_power)
        response.success = 1 # indicate success
        return response

    def stop_callback(self, request, response) -> None:
        """This service request stops the offloading belt."""
        self.stop()
        response.success = 1 # indicate success
        return response

    def toggle_callback(self, request, response) -> None:
        """This service request toggles the offloading belt."""
        self.toggle(request.drum_belt_power, request.conveyor_belt_power)
        response.success = 1 # indicate success
        return response
    

def main(args=None):
    """The main function."""
    rclpy.init(args=args)
    print("Initializing the Conveyor subsystem!")

    node = ConveyorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
