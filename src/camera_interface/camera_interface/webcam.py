#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_node')
        self.publisher_ = self.create_publisher(Image, '/webcam/image_raw', 10)  
        self.publisher1_ = self.create_publisher(Image, '/webcam/image_raw_1', 10)  
        self.publisher2_ = self.create_publisher(Image, '/webcam/image_raw_2', 10)  
        self.publisher3_ = self.create_publisher(Image, '/webcam/image_raw_3', 10)  
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        
    def timer_callback(self):
        msg = Image()

        if not self.cap.isOpened():
            self.get_logger().error('Error: Could not open webcam.')
            self.destroy_node()
        
        success, frame = self.cap.read()

        # If frame is not read successfully, break the loop
        if not success:
            self.get_logger().warn("Error: Could not read frame.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        self.get_logger().info('Publishing: laptop webcam frame')
        
        self.publisher_.publish(msg)
        self.publisher1_.publish(self.bridge.cv2_to_imgmsg(cv2.flip(frame, 0), encoding='bgr8'))
        self.publisher2_.publish(self.bridge.cv2_to_imgmsg(cv2.flip(frame, 1), encoding='bgr8'))
        self.publisher3_.publish(self.bridge.cv2_to_imgmsg(cv2.flip(frame, -1), encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()

    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()