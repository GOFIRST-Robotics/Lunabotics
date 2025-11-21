import os
from ament_index_python.resources import get_resource

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import QWidget



import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rclpy.executors import SingleThreadedExecutor


class CameraWidget(QWidget):
    def __init__(self, pkg_name="rqt_camera", ui_filename="cameras.ui"):
        super().__init__()

        # Load .ui
        _, package_path = get_resource("packages", pkg_name)
        ui_path = os.path.join(package_path, "share", pkg_name, "resource", ui_filename)
        loadUi(ui_path, self)
        self.setObjectName("CameraWidget")

        # Get widgets directly from UI (no need for findChild)
        self.topic_combo = self.topic_combo
        self.refresh_button = self.refresh_button
        self.camera_display = self.camera_display

        # ROS node (rqt already initialized RCL)
        self.node = rclpy.create_node("rqt_camera_viewer")

        self.bridge = CvBridge()
        self.subscription = None
        self.available_topics = []

        # Connect UI signals
        self.refresh_button.clicked.connect(self.refresh_topics)
        self.topic_combo.currentIndexChanged.connect(self.change_topic)
        
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        # qt timer for set ms intervals
        self._qt_timer = self.startTimer(1)

        # Initial topic scan
        self.refresh_topics()

    # -----------------------------
    # Topic scanning
    # -----------------------------
    def refresh_topics(self):
        topics = self.node.get_topic_names_and_types()
        image_topics = [
            name for name, types in topics
            if any("sensor_msgs/msg/Image" in t for t in types)
        ]

        self.available_topics = sorted(image_topics)
        self.topic_combo.clear()
        self.topic_combo.addItems(self.available_topics)

    # -----------------------------
    # Subscribe to new topic
    # -----------------------------
    def change_topic(self):
        topic = self.topic_combo.currentText()
        if not topic:
            return

        print(f"[rqt_camera] Subscribing to: {topic}")

        if self.subscription:
            self.node.destroy_subscription(self.subscription)


        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.node.create_subscription(
            Image,
            topic,
            self.image_callback,
            qos
        )

    # -----------------------------
    # Receive images
    # -----------------------------
    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            print("[rqt_camera] cv_bridge error:", e)
            return

        h, w, _ = cv_img.shape
        qimg = QImage(cv_img.data, w, h, w * 3, QImage.Format_RGB888)

        pix = QPixmap.fromImage(qimg)
        self.camera_display.setPixmap(
            pix.scaled(
                self.camera_display.width(),
                self.camera_display.height(),
                Qt.KeepAspectRatio,
            )
        )

    def timerEvent(self, event):
        self.executor.spin_once(timeout_sec=0)

    # -----------------------------
    # Shutdown
    # -----------------------------
    def shutdown(self):
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
        self.node.destroy_node()
