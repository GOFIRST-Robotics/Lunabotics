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

states = RobotStates()

class DS_Subscriber(Node):

    def __init__(self):
        super().__init__('ds_subscriber')
        self.subscription = self.create_subscription(String, 'robot_command', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        # Switch our current state if a switch has been requested
        if msg.data == states.teleop.value:
            states.current_state = states.teleop
        elif msg.data == states.auto.value:
            states.current_state = states.auto
        elif msg.data == states.dig.value:
            states.current_state = states.dig
        elif msg.data == states.halt.value:
            states.current_state = states.halt

        # Log the robot's current state
        self.get_logger().info('Current State' % states.current_state)


def main(args=None):
    rclpy.init(args=args)

    print('Hello from the rovr_control package!')
    print('Initial State:', states.current_state)

    ds_subscriber = DS_Subscriber()

    rclpy.spin(ds_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ds_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
