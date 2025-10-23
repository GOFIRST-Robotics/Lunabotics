
# Import the ROS 2 module
import rclpy
from rclpy.node import Node
from PIL import Image, ImageDraw, ImageFont
from StreamDeck.DeviceManager import DeviceManager, StreamDeckMini
from StreamDeck.ImageHelpers import PILHelper
import os

class StreamDeckNode(Node):

    ASSETS_PATH = "/workspaces/isaac_ros-dev/src/rovr_control/resource/"
    
    def __init__(self) -> None:
        super().__init__("StreamDeckNode")
        print("Hello from StreamDeckNode!")
        streamdecks = DeviceManager().enumerate()
        print("Found {} Stream Deck(s).\n".format(len(streamdecks)))
        self.deck: StreamDeckMini = streamdecks[0]
        self.deck.open()
        self.deck.reset()
        for key in range(self.deck.key_count()):
            self.update_key_image(key)
        self.deck.set_key_callback(self.key_change_callback)

    def key_change_callback(self, key, state):
        print("Deck {} Key {} = {}".format(self.deck.DECK_TYPE, key, state), flush=True)

    def update_key_image(self, key):
        # Generate the custom key with the requested image and label.
        image = self.render_key_image("test.png", "Hi!")

        # Use a scoped-with on the deck to ensure we're the only thread using it
        # right now.
        with self.deck:
            # Update requested key with the generated image.
            self.deck.set_key_image(key, image)

    def render_key_image(self, icon_filename, label_text):
        # Resize the source image asset to best-fit the dimensions of a single key,
        # leaving a margin at the bottom so that we can draw the key title
        # afterwards.
        icon = Image.open(os.path.join(self.ASSETS_PATH, icon_filename))
        image = PILHelper.create_scaled_key_image(self.deck, icon, margins=[0, 0, 20, 0])

        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default()
        draw.text((image.width / 2, image.height - 5), text=label_text, font=font, anchor="ms", fill="white")

        return PILHelper.to_native_key_format(self.deck, image)

def main(args=None):
    rclpy.init(args=args)

    stream_deck = StreamDeckNode()

    rclpy.spin(stream_deck)

    stream_deck.deck.close()
    stream_deck.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
