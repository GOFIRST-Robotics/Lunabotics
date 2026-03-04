import sys
import threading
import time
import math
import numpy as np
import av  # PyAV

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QWidget,
    QListWidget, QListWidgetItem,
    QGridLayout, QSizePolicy  # <--- Added QSizePolicy import
)
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, Signal, QObject

# -------------------- Dark Theme --------------------

dark_stylesheet = """
QWidget { background-color: #121212; color: #e0e0e0; }
QPushButton { background-color: #1f1f1f; border: 1px solid #333; padding: 6px; }
QPushButton:hover { background-color: #333; }
QListWidget { background-color: #1c1c1c; border: 1px solid #333; }
"""

# -------------------- Decoding Logic --------------------

class ImageSignal(QObject):
    image_ready = Signal(object)

class StreamDecoder:
    """Stateful decoder using PyAV for AV1/HEVC/H264 streams."""
    def __init__(self, codec_name='av1'):
        try:
            self.codec = av.CodecContext.create(codec_name, 'r')
        except Exception as e:
            print(f"Error creating codec {codec_name}: {e}")
            self.codec = None

    def decode(self, binary_data):
        if not self.codec:
            return None
        
        try:
            packets = self.codec.parse(binary_data)
            for packet in packets:
                frames = self.codec.decode(packet)
                for frame in frames:
                    return frame.to_ndarray(format='rgb24')
        except Exception as e:
            print(f"Decode error: {e}")
        return None

# -------------------- ROS Node --------------------

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('multi_camera_qt_interface')
        self.bridge = CvBridge()
        self.subscriptions_map = {}
        self.image_signals = {}
        self.decoders = {}
        self.last_frame_time = {}
        self.max_fps = 60.0

    def subscribe(self, topic, msg_type_str):
        if topic in self.subscriptions_map:
            return

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.last_frame_time[topic] = 0.0

        # Map string type to class type
        if 'CompressedImage' in str(msg_type_str):
            msg_type = CompressedImage
        else:
            msg_type = Image

        self.subscriptions_map[topic] = self.create_subscription(
            msg_type,
            topic,
            lambda msg, t=topic: self.callback(msg, t),
            qos
        )

    def unsubscribe(self, topic):
        if topic in self.subscriptions_map:
            self.destroy_subscription(self.subscriptions_map[topic])
            del self.subscriptions_map[topic]
            del self.last_frame_time[topic]
            if topic in self.decoders:
                del self.decoders[topic]

    def callback(self, msg, topic):
        if topic not in self.image_signals:
            return

        now = time.time()
        if now - self.last_frame_time.get(topic, 0.0) < 1.0 / self.max_fps:
            return
        self.last_frame_time[topic] = now

        cv_img = None
        try:
            if isinstance(msg, CompressedImage):
                if topic not in self.decoders:
                    fmt = msg.format.lower()
                    codec = 'hevc' if 'hevc' in fmt else 'av1' if 'av1' in fmt else 'h264'
                    self.decoders[topic] = StreamDecoder(codec)
                
                cv_img = self.decoders[topic].decode(bytes(msg.data))
                
            elif isinstance(msg, Image):
                if "av1" in msg.encoding.lower():
                    if topic not in self.decoders:
                        self.decoders[topic] = StreamDecoder('av1')
                    cv_img = self.decoders[topic].decode(bytes(msg.data))
                else:
                    cv_img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

            if cv_img is not None:
                signal = self.image_signals.get(topic)
                if signal:
                    signal.image_ready.emit(cv_img)
                    
        except Exception as e:
            self.get_logger().error(f"Processing error on {topic}: {e}")

# -------------------- UI Components --------------------

class CameraWidget(QWidget):
    def __init__(self, topic):
        super().__init__()
        self.topic = topic
        self.last_frame = None

        self.label = QLabel("Waiting for Stream...")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("background-color: #000; color: #555;")
        
        # FIX 1: Use QSizePolicy instead of QWidget.SizePolicy
        self.label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        
        title = QLabel(topic)
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 10px; color: #888; background: #1a1a1a; padding: 2px;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.label, 1)
        layout.addWidget(title)

    def update_image(self, cv_img):
        self.last_frame = cv_img 
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        
        qimg = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
        
        pix = QPixmap.fromImage(qimg)
        self.label.setPixmap(pix.scaled(
            self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))

    def resizeEvent(self, event):
        if self.last_frame is not None:
            self.update_image(self.last_frame)
        super().resizeEvent(event)

class MultiCameraWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.camera_widgets = {}
        self.setWindowTitle("ROS2 Stream Viewer")

        main_widget = QWidget()
        self.layout = QHBoxLayout(main_widget)
        
        side_panel = QWidget()
        side_layout = QVBoxLayout(side_panel)
        self.topic_list = QListWidget()
        self.topic_list.itemChanged.connect(self.topic_toggled)
        
        btn_refresh = QPushButton("Refresh Topics")
        btn_refresh.clicked.connect(self.refresh_topics)
        
        side_layout.addWidget(QLabel("Available Topics:"))
        side_layout.addWidget(self.topic_list)
        side_layout.addWidget(btn_refresh)
        
        self.grid_widget = QWidget()
        self.grid_layout = QGridLayout(self.grid_widget)
        self.grid_layout.setContentsMargins(0,0,0,0)
        
        self.layout.addWidget(side_panel, 1)
        self.layout.addWidget(self.grid_widget, 4)
        self.setCentralWidget(main_widget)
        
        self.refresh_topics()

    def refresh_topics(self):
        self.topic_list.blockSignals(True)
        checked = {self.topic_list.item(i).text() for i in range(self.topic_list.count()) 
                   if self.topic_list.item(i).checkState() == Qt.Checked}

        self.topic_list.clear()
        
        try:
            topic_names_and_types = self.ros_node.get_topic_names_and_types()
        except Exception as e:
            print(f"Error fetching topics: {e}")
            topic_names_and_types = []

        for name, types in topic_names_and_types:
            # Check for relevant message types
            if any(t in ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'] for t in types):
                item = QListWidgetItem(name)
                item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                item.setCheckState(Qt.Checked if name in checked else Qt.Unchecked)
                # Store the type string for later use
                item.setData(Qt.UserRole, types[0]) 
                self.topic_list.addItem(item)
        
        self.topic_list.blockSignals(False)

    def topic_toggled(self, item):
        topic = item.text()
        msg_type = item.data(Qt.UserRole)
        
        if item.checkState() == Qt.Checked:
            self.add_camera(topic, msg_type)
        else:
            self.remove_camera(topic)

    def add_camera(self, topic, msg_type):
        if topic in self.camera_widgets: return
        
        widget = CameraWidget(topic)
        sig = ImageSignal()
        sig.image_ready.connect(widget.update_image)
        self.ros_node.image_signals[topic] = sig
        
        self.ros_node.subscribe(topic, msg_type)
        self.camera_widgets[topic] = widget
        self.rebuild_grid()

    def remove_camera(self, topic):
        if topic in self.camera_widgets:
            self.ros_node.unsubscribe(topic)
            if topic in self.ros_node.image_signals:
                del self.ros_node.image_signals[topic]
            
            widget = self.camera_widgets.pop(topic)
            self.grid_layout.removeWidget(widget)
            widget.deleteLater()
            self.rebuild_grid()

    def rebuild_grid(self):
        for i in reversed(range(self.grid_layout.count())): 
            item = self.grid_layout.itemAt(i)
            if item.widget():
                item.widget().setParent(None)

        widgets = list(self.camera_widgets.values())
        if not widgets: return

        count = len(widgets)
        cols = math.ceil(math.sqrt(count))
        
        for i, widget in enumerate(widgets):
            row, col = divmod(i, cols)
            self.grid_layout.addWidget(widget, row, col)

# -------------------- Execution --------------------

def main():
    rclpy.init()
    
    node = UserInterfaceNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    app = QApplication(sys.argv)
    app.setStyleSheet(dark_stylesheet)
    
    gui = MultiCameraWindow(node)
    gui.resize(1280, 720)
    gui.show()
    
    try:
        # FIX 2: Do not wrap in sys.exit() yet to allow clean shutdown block
        app.exec()
    finally:
        # FIX 3: Check if rclpy is still okay before shutting down
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()