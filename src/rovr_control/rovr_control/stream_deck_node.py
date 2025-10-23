
# Import the ROS 2 module
import rclpy
import time
from rclpy.node import Node
from StreamDeck.DeviceManager import DeviceManager


class StreamDeckNode(Node):
    
    def __init__(self) -> None:
        super().__init__("StreamDeckNode")
        print("Hello from StreamDeckNode!")
        streamdecks = DeviceManager().enumerate()
        print("Found {} Stream Deck(s).\n".format(len(streamdecks)))

        
def main(args=None):
    rclpy.init(args=args)

    stream_deck = StreamDeckNode()

    rclpy.spin(stream_deck)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stream_deck.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
