
# Import the ROS 2 module
import os
import time
from queue import Queue

import rclpy
from PIL import Image, ImageDraw, ImageFont
from rclpy.node import Node
from StreamDeck.DeviceManager import DeviceManager, StreamDeckMini
from StreamDeck.ImageHelpers import PILHelper

from rovr_interfaces.msg import StreamDeckState


class StreamDeckNode(Node):

    ASSETS_PATH = "/workspaces/isaac_ros-dev/src/rovr_control/resource/"
    button_states: list[bool] = [False] * StreamDeckMini.KEY_COUNT

    def __init__(self) -> None:
        super().__init__("StreamDeckNode")
        self.publisher = self.create_publisher(StreamDeckState, 'control/stream_deck', 10)
        msg = StreamDeckState()
        msg.button_states = self.button_states
        self.publisher.publish(msg)
        
        self.queue = Queue()

        streamdecks = DeviceManager().enumerate()
        while len(streamdecks) == 0:
            print("No Stream Decks found. Retrying...")
            time.sleep(1)
            streamdecks = DeviceManager().enumerate()
        print("Found Stream Deck.")
        self.deck: StreamDeckMini = streamdecks[0]
        self.deck.open()
        self.deck.reset()
        for key in range(self.deck.key_count()):
            self.set_key_image(key)
        self.deck.set_key_callback(self.key_change_callback)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self) -> None:
        while not self.queue.empty():
            msg = self.queue.get()
            self.publisher.publish(msg)

    def key_change_callback(self, _, key: int, state: bool) -> None:
        self.button_states[key] = state
        msg = StreamDeckState()
        msg.button_states = self.button_states
        self.queue.put(msg)

    def set_key_image(self, key: int) -> None:
        image = self.render_image("test.png", "Hi!")
        with self.deck:
            self.deck.set_key_image(key, image)

    def render_image(self, icon_filename: str, label_text: str) -> bytes:
        # Resize the source image asset to best-fit the dimensions of a single key,
        # leaving a margin at the bottom so that we can draw the key title
        # afterwards.
        icon = Image.open(os.path.join(self.ASSETS_PATH, icon_filename))
        image = PILHelper.create_scaled_key_image(self.deck, icon, margins=[0, 0, 20, 0])
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default()
        draw.text((image.width / 2, image.height - 5), text=label_text, font=font, anchor="ms", fill="white")

        return PILHelper.to_native_key_format(self.deck, image)

def main(args=None) -> None:
    rclpy.init(args=args)

    stream_deck = StreamDeckNode()

    try:
        rclpy.spin(stream_deck)
    finally:
        stream_deck.deck.close()
    stream_deck.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()