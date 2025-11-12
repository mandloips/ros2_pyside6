import sys
import threading
import rclpy
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from gui import AppWindow
from ros_gui import RosGui

# running rclpy.spin in a separate thread
def ros_spin(node, shutdown_event):
    try:
        while not shutdown_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)  # Non-blocking spin_once to allow shutdown_event check
    finally:
        node.destroy_node()
        rclpy.shutdown()

# setting up ROS node and PySide6 GUI window
def main(args=None):
    rclpy.init(args=args)
    node = RosGui()

    shutdown_event = threading.Event()

    ros_thread = threading.Thread(target=ros_spin, args=(node, shutdown_event))
    ros_thread.start()

    app = QApplication(sys.argv)
    window = AppWindow(node, shutdown_event)
    window.show()

    sys.exit(app.exec())

    ros_thread.join()

    print("ROS Node and GUI have been shut down.")

if __name__ == '__main__':
    main()
