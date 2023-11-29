import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Bool
import serial
import struct

class read_serial(Node):
    def __init__(self):
        super().__init__("read_serial")
        self.frontLeftEncoder = self.create_publisher(Int8, "frontLeftEncoder", 10)
        self.frontRightEncoder = self.create_publisher(Int8, "frontRightEncoder", 10)
        self.backLeftEncoder  = self.create_publisher(Int8, "backLeftEncoder", 10)
        self.backRightEncoder = self.create_publisher(Int8, "backRightEncoder", 10)
        self.topLimitSwitch = self.create_publisher(Int8, "topLimitSwitch", 10)
        self.bottomLimitSwitch = self.create_publisher(Int8, "bottomLimitSwitch", 10)
        self.timer = self.create_timer(0.1, self.read_data)
        try:
            self.arduino = serial.Serial("/dev/name", 9600)
        except Exception as e:
            self.get_logger().fatal(f"Error connecting to serial: {e}")
        
    def read_data(self):
        data = self.arduino.read()
        decoded = struct.unpack('iiiibb', data)
        msg = Int8()
        msg.data = decoded[0]
        self.frontLeftEncoder.publish(msg)
        msg.data = decoded[1]
        self.frontRightEncoder.publish(msg)
        msg.data = decoded[2]
        self.backLeftEncoder.publish(msg)
        msg.data = decoded[3]
        self.backRightEncoder.publish(msg)
        msg = Bool()
        msg.data = decoded[4]
        self.topLimitSwitch.publish(msg)
        msg.data = decoded[5]
        self.bottomLimitSwitch.publish(msg)

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
