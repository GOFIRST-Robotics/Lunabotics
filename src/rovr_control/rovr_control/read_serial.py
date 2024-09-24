import rclpy
from rclpy.node import Node
from rovr_interfaces.msg import LimitSwitches

import serial
import struct
import time


class read_serial(Node):
    def __init__(self):
        super().__init__("read_serial")

        self.limitSwitchesPub = self.create_publisher(LimitSwitches, "limitSwitches", 10)

        try:
            self.arduino = serial.Serial("/dev/ttyACM0", 9600)
            time.sleep(1)  # https://stackoverflow.com/questions/7266558/pyserial-buffer-wont-flush
            self.arduino.read_all()
        except Exception as e:
            self.get_logger().fatal(f"Error connecting to serial: {e}")
            self.destroy_node()
            return

        while True:
            if self.arduino is None:
                self.get_logger().fatal("Killing read_serial node")
                self.destroy_node()
                return
            data = self.arduino.read(10)  # Pause until 10 bytes are read
            decoded = struct.unpack("??", data)  # Use h for each int because arduino int is 2 bytes

            msg = LimitSwitches()
            msg.digger_top_limit_switch = decoded[0]
            msg.digger_bottom_limit_switch = decoded[1]
            msg.dumper_top_limit_switch = decoded[2]
            msg.dumper_bottom_limit_switch = decoded[3]
            self.limitSwitchesPub.publish(msg)


def main(args=None):
    """The main function."""
    rclpy.init(args=args)

    node = read_serial()
    node.get_logger().info("Starting serial reader")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# This code does NOT run if this file is imported as a module
if __name__ == "__main__":
    main()
