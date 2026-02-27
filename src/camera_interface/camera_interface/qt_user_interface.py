import sys
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QWidget,
    QListWidget, QListWidgetItem,
    QGridLayout, QSizePolicy
)
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, Signal, QObject

# -------------------- Dark Theme --------------------

dark_stylesheet = """
QWidget {
    background-color: #121212;
    color: #e0e0e0;
}
QPushButton {
    background-color: #1f1f1f;
    border: 1px solid #333;
    padding: 6px;
}
QListWidget {
    background-color: #1c1c1c;
}
"""

# -------------------- Qt Signal --------------------

class ImageSignal(QObject):
    image_ready = Signal(object)

# -------------------- ROS Node --------------------

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('multi_camera_qt_interface')
        self.bridge = CvBridge()
        self.camera_subscriptions = {}
        self.image_signals = {}
        self.last_frame = {}
        self.max_fps = 30.0

    def subscribe(self, topic):
        if topic in self.camera_subscriptions:
            return

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.last_frame[topic] = 0.0

        self.camera_subscriptions[topic] = self.create_subscription(
            Image,
            topic,
            lambda msg, t=topic: self.callback(msg, t),
            qos
        )

    def unsubscribe(self, topic):
        if topic in self.camera_subscriptions:
            self.destroy_subscription(self.camera_subscriptions[topic])
            del self.camera_subscriptions[topic]
            del self.last_frame[topic]
            del self.image_signals[topic]

    def callback(self, msg, topic):
        # Topic removed while callback was queued
        if topic not in self.image_signals:
            return

        now = time.time()
        if now - self.last_frame.get(topic, 0.0) < 1.0 / self.max_fps:
            return

        self.last_frame[topic] = now

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

        signal = self.image_signals.get(topic)
        if signal is not None:
            signal.image_ready.emit(cv_img)

# -------------------- Camera Widget --------------------

class CameraWidget(QWidget):
    def __init__(self, topic):
        super().__init__()
        self.topic = topic

        self.label = QLabel("Waiting for image...")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Ignored)
        self.label.setMinimumSize(1, 1)

        title = QLabel(topic)
        title.setAlignment(Qt.AlignCenter)
        title.setFixedHeight(18)
        title.setStyleSheet("font-size:10px; border-top:1px solid #333")
        
        # ---- Image container ----
        image_container = QWidget()
        image_layout = QVBoxLayout()
        image_layout.setContentsMargins(0, 0, 0, 0)
        image_layout.addWidget(self.label)
        # image_layout.addWidget(title, 0)
        image_container.setLayout(image_layout)

        image_container.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Ignored
        )
    
        # ---- Main layout ----
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        layout.addWidget(title, 1)            # stays attached
        layout.addWidget(image_container, 0)  # stretches

        self.setLayout(layout)

    def update_image(self, cv_img):
        self.last_frame = cv_img   # cache frame

        h, w, ch = cv_img.shape
        qimg = QImage(cv_img.data, w, h, ch * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)

        self.label.setPixmap(
            pix.scaled(
                self.label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
        )

    def resizeEvent(self, event):
        if hasattr(self, "last_frame"):
            self.update_image(self.last_frame)

# -------------------- Main Window --------------------

class MultiCameraWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("ROS2 Multi-Camera Viewer")
        self.ros_node = ros_node

        self.camera_widgets = {}

        main_widget = QWidget()
        main_layout = QHBoxLayout()

        # ---- Sidebar ----
        side_layout = QVBoxLayout()
        self.refresh_btn = QPushButton("Refresh Topics")
        self.refresh_btn.clicked.connect(self.refresh_topics)

        self.topic_list = QListWidget()
        self.topic_list.itemChanged.connect(self.topic_toggled)

        side_layout.addWidget(self.topic_list)
        side_layout.addWidget(self.refresh_btn)

        # ---- Grid Area ----
        self.grid_widget = QWidget()
        self.grid_layout = QGridLayout()
        self.grid_widget.setLayout(self.grid_layout)
        self.grid_widget.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding
        )


        main_layout.addLayout(side_layout)
        main_layout.addWidget(self.grid_widget, 1)

        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        self.refresh_topics()

    # --------------------

    def refresh_topics(self):
        self.topic_list.blockSignals(True)
        self.topic_list.clear()

        topics = self.ros_node.get_topic_names_and_types()
        for name, types in topics:
            if "sensor_msgs/msg/Image" in types:
                item = QListWidgetItem(name)
                item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                item.setCheckState(Qt.Unchecked)
                self.topic_list.addItem(item)

        self.topic_list.blockSignals(False)

    # --------------------

    def topic_toggled(self, item):
        topic = item.text()
        checked = item.checkState() == Qt.Checked

        if checked:
            self.add_camera(topic)
        else:
            self.remove_camera(topic)

    # --------------------

    def add_camera(self, topic):
        if topic in self.camera_widgets:
            return

        widget = CameraWidget(topic)
        signal = ImageSignal()
        signal.image_ready.connect(widget.update_image)

        self.ros_node.image_signals[topic] = signal
        self.ros_node.subscribe(topic)

        self.camera_widgets[topic] = widget
        self.rebuild_grid()

    def remove_camera(self, topic):
        if topic not in self.camera_widgets:
            return

        self.ros_node.unsubscribe(topic)

        widget = self.camera_widgets[topic]
        widget.setParent(None)
        widget.deleteLater()

        del self.camera_widgets[topic]
        self.rebuild_grid()


    # --------------------
    
    def rebuild_grid(self):
        # Clear previous stretches to prevent "dead space"
        for i in range(self.grid_layout.rowCount()):
            self.grid_layout.setRowStretch(i, 0)
        for i in range(self.grid_layout.columnCount()):
            self.grid_layout.setColumnStretch(i, 0)

        # Properly clear the layout
        while self.grid_layout.count():
            item = self.grid_layout.takeAt(0)
            if item.widget():
                item.widget().hide()
                
        widgets = list(self.camera_widgets.values())
        count = len(widgets)

        if count == 0:
            return

        cols = math.ceil(math.sqrt(count))
        rows = math.ceil(count / cols)

        index = 0
        for r in range(rows):
            for c in range(cols):
                if index >= count:
                    break
                
                widget = widgets[index]
                self.grid_layout.addWidget(widget, r, c)
                
                # Ensure it's visible and active
                widget.setVisible(True) 
                widget.update() # Triggers a repaint of the camera frame
                
                index += 1

        for r in range(rows):
            self.grid_layout.setRowStretch(r, 1)
        for c in range(cols):
            self.grid_layout.setColumnStretch(c, 1)

        self.grid_layout.setContentsMargins(0, 0, 0, 0)
        self.grid_layout.setSpacing(1)
        


# -------------------- Main --------------------

def main():
    rclpy.init()

    app = QApplication(sys.argv)
    app.setStyleSheet(dark_stylesheet)

    ros_node = UserInterfaceNode()

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True
    )
    ros_thread.start()

    win = MultiCameraWindow(ros_node)
    # win.resize(1200, 800)
    win.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
