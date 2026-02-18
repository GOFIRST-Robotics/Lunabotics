#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import subprocess
import threading
import time

class GpuWebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_gpu_node')
        
        # Configuration
        self.device_id = 0
        self.fps = 30
        
        # 1. Setup Camera with V4L2 Backend
        # 'cv2.CAP_V4L2' prevents the GStreamer "Internal data stream error"
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        
        # CRITICAL: Force MJPEG. 
        # Most USB cams cannot do 1280x720 @ 30fps in raw YUYV format (bandwidth limit).
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # Force Resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Read actual parameters (in case camera rejected our request)
        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 640
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480
        
        # NVENC requires even dimensions
        self.width = int(w) if int(w) % 2 == 0 else int(w) - 1
        self.height = int(h) if int(h) % 2 == 0 else int(h) - 1
        
        # Codec Settings
        self.codec = 'hevc_nvenc'  # NVIDIA Hardware Encoder
        self.stream_fmt = 'hevc'   # Raw HEVC stream
        
        # 2. Setup FFmpeg
        self.ffmpeg_cmd = [
            'ffmpeg', '-y', 
            '-hide_banner', '-loglevel', 'error',
            '-f', 'rawvideo', 
            '-vcodec', 'rawvideo',
            '-s', f'{self.width}x{self.height}', 
            '-pix_fmt', 'bgr24', 
            '-r', str(self.fps),
            '-i', '-',          # Input from pipe
            '-c:v', self.codec,
            '-preset', 'p1',    # Low latency preset
            '-tune', 'ull',     # Ultra Low Latency
            '-zerolatency', '1',
            '-g', '30',         # Keyframe every 1s
            '-bf', '0',         # No B-frames
            '-f', self.stream_fmt, 
            '-'                 # Output to pipe
        ]

        try:
            self.process = subprocess.Popen(
                self.ffmpeg_cmd, 
                stdin=subprocess.PIPE, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                bufsize=0  # Unbuffered IO
            )
        except FileNotFoundError:
            self.get_logger().error("FFmpeg not found. Ensure ffmpeg is installed.")
            return

        self.publisher_ = self.create_publisher(CompressedImage, '/webcam/gpu_stream', 10)
        
        # 3. Start Threads
        self.input_timer = self.create_timer(1.0 / self.fps, self.feed_encoder)
        
        self.output_thread = threading.Thread(target=self.read_encoded_stream, daemon=True)
        self.output_thread.start()
        
        self.get_logger().info(f"Streaming {self.width}x{self.height} via {self.codec} (MJPG Input)...")

    def feed_encoder(self):
        """Capture frame and push to FFmpeg stdin"""
        if not self.cap.isOpened():
            return
            
        ret, frame = self.cap.read()
        if ret:
            # Resize if the camera ignores our resolution request
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))

            try:
                self.process.stdin.write(frame.tobytes())
                self.process.stdin.flush()
            except BrokenPipeError:
                self.get_logger().error("FFmpeg stdin broken pipe")
                rclpy.shutdown()
        else:
            self.get_logger().warn("Camera frame read failed.")

    def read_encoded_stream(self):
        """Continuously read from FFmpeg stdout and publish"""
        # Buffer size large enough for 1080p frames to prevent artifacting
        chunk_size = 256 * 1024 
        
        while rclpy.ok() and self.process.poll() is None:
            try:
                # Read whatever is available
                data = self.process.stdout.read(chunk_size)
                
                if data:
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format = self.stream_fmt
                    msg.data = data
                    
                    self.publisher_.publish(msg)
                else:
                    time.sleep(0.002)
            except Exception as e:
                self.get_logger().error(f"Read error: {e}")
                break

    def destroy_node(self):
        if self.process:
            self.process.kill()
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpuWebcamPublisher()
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