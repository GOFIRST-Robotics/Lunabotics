# ROS Modules
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Other Modules
from statemachine import StateMachine, State

# Define our various robot states here
class RobotStates(StateMachine):
    teleop = State('Teleop Drive', initial=True)
    auto = State('Autonomous Drive')
    dig = State('Autonomous Dig')
    halt = State('Halt')

    teleop_to_auto = teleop.to(auto)
    teleop_to_dig = teleop.to(dig)
    teleop_to_halt = teleop.to(halt)

def main():
    print('Hi from the rovr_control node!')

    print('State Machine test')
    states = RobotStates()
    print('Current State:', states.current_state)
    states.current_state = states.auto
    print('Current State:', states.current_state)


if __name__ == '__main__':
    main()
