import rclpy
from rclpy.node import Node
from rovr_interfaces.msg import Potentiometers
from rovr_interfaces.msg import LimitSwitches
from std_srvs.srv import SetBool, Trigger

import serial
import struct
import time


class read_serial(Node):
    def __init__(self):
        super().__init__("read_serial")

        self.potentiometerPub = self.create_publisher(Potentiometers, "potentiometers", 10)
        self.LimitSwitchPub = self.create_publisher(LimitSwitches, "Limit Switches", 10)

        # Services to control the relay-driven agitator motor
        self.srv_bigonoff = self.create_service(SetBool, "big_agitator_on_off", self.big_on_off_callback)
        self.srv_bigtoggle = self.create_service(Trigger, "big_agitator_toggle", self.big_toggle_callback)

        self.srv_smallonoff = self.create_service(SetBool, "small_agitator_on_off", self.small_on_off_callback)
        self.srv_smalltoggle = self.create_service(Trigger, "small_agitator_toggle", self.small_toggle_callback)

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

        self.bigAgitatorOn = False
        self.smallAgitatorOn = False

    def timer_callback(self):
        if self.arduino is None:
            self.get_logger().fatal("Killing read_serial node")
            self.destroy_node()
            return
        data = self.arduino.read(4)  # Pause until 4 bytes are read
        decoded = struct.unpack("hh", data)  # Use h for integers and ? for booleans


        msg = Potentiometers()
        msg.left_motor_pot = decoded[0]
        msg.right_motor_pot = decoded[1]
        self.potentiometerPub.publish(msg)
        self.lastMsg = msg
        
        msg = LimitSwitches()
        msg.bottom_limit_switch = decoded[2]
        msg.top_limit_switch = decoded[3]
        self.LimitSwitchPub.publish(msg)
        self.lastMsg = msg



    def big_on_off_callback(self, request, response):
        # request.data == True  → ON, False → OFF
        cmd = b"1" if request.data else b"0"
        self.arduino.write(cmd)
        response.success = True
        response.message = "Big agitator motor turned " + ("on" if request.data else "off")
        self.bigAgitatorOn = request.data
        return response

    def big_toggle_callback(self, request, response):
        if self.bigAgitatorOn:
            response2 = SetBool.Response()
            self.big_on_off_callback(SetBool.Request(data=False), response2)
        else:
            response2 = SetBool.Response()
            self.big_on_off_callback(SetBool.Request(data=True), response2)
        response.success = response2.success
        response.message = response2.message
        return response

    def small_on_off_callback(self, request, response):
        # request.data == True  → ON, False → OFF
        cmd = b"3" if request.data else b"2"
        self.arduino.write(cmd)
        response.success = True
        response.message = "Small agitator motor turned " + ("on" if request.data else "off")
        self.smallAgitatorOn = request.data
        return response

    def small_toggle_callback(self, request, response):
        if self.smallAgitatorOn:
            response2 = SetBool.Response()
            self.small_on_off_callback(SetBool.Request(data=False), response2)
        else:
            response2 = SetBool.Response()
            self.small_on_off_callback(SetBool.Request(data=True), response2)
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
