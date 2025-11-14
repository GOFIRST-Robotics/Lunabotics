import rclpy
from rclpy.node import Node
from rovr_interfaces.msg import Potentiometers
from std_srvs.srv import SetBool, Trigger

import serial
import struct
import time


class read_serial(Node):
    def __init__(self):
        super().__init__("read_serial")

        self.potentiometerPub = self.create_publisher(int, "potentiometer", 10)

        # Services to control the relay-driven agitator motor
        self.srv_onoff = self.create_service(SetBool, "motor_on_off", self.on_off_callback)
        self.srv_toggle = self.create_service(Trigger, "motor_toggle", self.toggle_callback)

        try:
            self.arduino = serial.Serial("/dev/ttyACM0", 9600)
            time.sleep(1)  # https://stackoverflow.com/questions/7266558/pyserial-buffer-wont-flush
            self.arduino.read_all()
        except Exception as e:
            self.get_logger().fatal(f"Error connecting to serial: {e}")
            self.destroy_node()
            return

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.lastMsg = Potentiometers()
        self.agitatorOn = False

    def timer_callback(self):
        if self.arduino is None:
            self.get_logger().fatal("Killing read_serial node")
            self.destroy_node()
            return
        data = self.arduino.read(4)  # Pause until 4 bytes are read
        decoded = struct.unpack("h", data)  # Use h for integers and ? for booleans

        
        self.potentiometerPub.publish(decoded[0])
        self.lastMsg = decoded[0]

    def on_off_callback(self, request, response):
        # request.data == True  → ON, False → OFF
        cmd = b"1" if request.data else b"0"
        self.arduino.write(cmd)
        response.success = True
        response.message = "Agitator motor turned " + ("on" if request.data else "off")
        self.agitatorOn = request.data
        return response

    def toggle_callback(self, request, response):
        if self.agitatorOn:
            response2 = SetBool.Response()
            self.on_off_callback(SetBool.Request(data=False), response2)
        else:
            response2 = SetBool.Response()
            self.on_off_callback(SetBool.Request(data=True), response2)
        response.success = response2.success
        response.message = response2.message
        return response


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
