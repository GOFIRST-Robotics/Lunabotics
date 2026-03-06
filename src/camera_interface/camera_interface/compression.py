#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CompressedImage
import cv2
import subprocess
import threading
import time

class VideoCompressionNode(Node):
    def __init__(self):
        super().__init__('video_compression_node')
        
        self.initalize_parameters()        
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().string_value
        
        # Determine camera source
        if self.device_id.isdigit():
            self.camera_source = int(self.device_id)
        else:
            self.camera_source = self.device_id
            
        self.fps = 30
        self.cap = None
        self.process = None
        self.is_running = True
        
        # Codec Settings        
        self.codec = 'hevc_nvenc'
        self.stream_fmt = 'hevc'
        
        self.publisher_ = self.create_publisher(CompressedImage, self.topic_name, 10)
        
        # Start the background thread for FFmpeg output
        self.output_thread = threading.Thread(target=self.read_encoded_stream, daemon=True)
        self.output_thread.start()
        
        # Timer for capturing frames
        self.input_timer = self.create_timer(1.0 / self.fps, self.feed_encoder)
        
        self.get_logger().info(f"Node started for device: {self.camera_source}")

    def open_camera(self):
        """Attempts to open the camera and configure it."""
        if self.cap is not None:
            self.cap.release()

        self.cap = cv2.VideoCapture(self.camera_source, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            return False

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 640
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480
        self.width = int(w) if int(w) % 2 == 0 else int(w) - 1
        self.height = int(h) if int(h) % 2 == 0 else int(h) - 1
        
        return True

    def setup_ffmpeg(self):
        """Initializes the FFmpeg subprocess."""
        if self.process:
            self.process.kill()

        ffmpeg_cmd = [
            'ffmpeg', '-y', '-hide_banner', '-loglevel', 'error',
            '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-s', f'{self.width}x{self.height}', '-pix_fmt', 'bgr24', 
            '-r', str(self.fps), '-i', '-', 
            '-c:v', self.codec, '-preset', 'p1', '-tune', 'ull',
            '-zerolatency', '1', '-g', '30', '-bf', '0',
            '-f', self.stream_fmt, '-'
        ]
        
        self.process = subprocess.Popen(
            ffmpeg_cmd, stdin=subprocess.PIPE, 
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
        )

    def feed_encoder(self):
        """Capture frame and push to FFmpeg. Handles reconnection if camera is lost."""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn(f"Camera {self.camera_source} non-existent. Retrying...", throttle_duration_sec=2.0)
            if self.open_camera():
                self.setup_ffmpeg()
                self.get_logger().info(f"Camera {self.camera_source} reconnected!")
            return

        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().error("Camera unplugged or read failed.")
            self.cap.release()
            return

        # Ensure correct dimensions for FFmpeg
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        try:
            if self.process and self.process.stdin:
                self.process.stdin.write(frame.tobytes())
                self.process.stdin.flush()
        except (BrokenPipeError, AttributeError):
            self.get_logger().error("FFmpeg pipe broken. Resetting...")
            self.setup_ffmpeg()

    def read_encoded_stream(self):
        """Continuously read from FFmpeg stdout and publish."""
        chunk_size = 256 * 1024 
        while rclpy.ok() and self.is_running:
            if self.process and self.process.poll() is None:
                try:
                    data = self.process.stdout.read(chunk_size)
                    if data:
                        msg = CompressedImage()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.format = self.stream_fmt
                        msg.data = data
                        self.publisher_.publish(msg)
                    else:
                        time.sleep(0.005)
                except Exception:
                    time.sleep(0.1)
            else:
                time.sleep(0.1)

    def initalize_parameters(self):
        self.declare_parameter('topic_name', value='', 
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('device_id', value='0', 
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

    def destroy_node(self):
        self.is_running = False
        if self.process:
            self.process.kill()
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoCompressionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()