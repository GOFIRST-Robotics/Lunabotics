#!/usr/bin/env python3
"""
Standalone script to run the GStreamer client widget UI without requiring the server.
This allows you to test the UI interface locally.
"""

import sys
import os
import rclpy
from rclpy.node import Node
from python_qt_binding.QtWidgets import QApplication
from gstreamer.client_widget import ClientWidget


def check_display():
    """Check if display is available, provide helpful error if not."""
    display = os.environ.get("DISPLAY")
    if not display:
        print("\n" + "=" * 70)
        print("ERROR: No display available!")
        print("=" * 70)
        print("\nTo run the UI in Docker, you have a few options:\n")
        print("Option 1: Use Xvfb (Virtual Framebuffer) - Recommended")
        print("  Install: sudo apt-get update && sudo apt-get install -y xvfb")
        print("  Run:     xvfb-run -a ros2 run gstreamer standalone_client\n")
        print("Option 2: Use X11 forwarding (if on Linux host)")
        print("  Run container with: -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix")
        print("  Then: ros2 run gstreamer standalone_client\n")
        print("Option 3: Use Qt offscreen platform (no visible UI)")
        print("  Run: QT_QPA_PLATFORM=offscreen ros2 run gstreamer standalone_client")
        print("  (Note: This won't show the UI, but will test if code runs)\n")
        print("=" * 70 + "\n")
        return False
    return True


def main():
    # Check for display before initializing Qt
    if not check_display():
        # Try to use offscreen platform as fallback
        if "QT_QPA_PLATFORM" not in os.environ:
            print("Attempting to use offscreen platform...")
            os.environ["QT_QPA_PLATFORM"] = "offscreen"
        else:
            sys.exit(1)

    # Initialize ROS 2
    rclpy.init(args=sys.argv)

    # Create a minimal ROS node (required by ClientWidget)
    node = Node("standalone_client_node")

    # Create Qt application
    app = QApplication(sys.argv)

    # Create and show the client widget
    widget = ClientWidget(node)
    widget.setWindowTitle("Camera Control (Standalone)")

    # Only show widget if we have a real display (not offscreen)
    if os.environ.get("QT_QPA_PLATFORM") != "offscreen":
        widget.show()
    else:
        print("Running in offscreen mode - UI created but not visible.")
        print("To see the UI, use Xvfb: xvfb-run -a ros2 run gstreamer standalone_client")

    # Start ROS executor in a separate thread or use Qt's event loop
    # We'll use a timer to spin ROS periodically
    from python_qt_binding.QtCore import QTimer

    def spin_ros():
        rclpy.spin_once(node, timeout_sec=0.01)

    timer = QTimer()
    timer.timeout.connect(spin_ros)
    timer.start(10)  # Spin every 10ms

    print("Camera Control UI started (standalone mode)")
    print("Note: Service calls will fail without the server running, but you can test the UI")

    # Run the Qt event loop
    exit_code = app.exec_()

    # Cleanup
    timer.stop()
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
