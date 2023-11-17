import sys
import time

import math
import numpy as np
import pyqtgraph as pg
import rclpy
from geometry_msgs.msg import Quaternion
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QVBoxLayout, QWidget, QPushButton
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtCore import Qt

from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from fiducial_msgs.msg import FiducialMarkerData


ENABLE_GUI = True
PLOT_WINDOW_SIZE_S = 10  # time in seconds
CAMERA_FRAME_ID = "left_hand_camera"
CHILD_FRAME_ID_PREFIX = "fiducial_marker_"

class FiducialMarkerFVD(Node):
    def __init__(self):
        super().__init__("fiducial_marker_fvd_node")
        self.subscription = self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)
        self.subscription  # prevent unused variable warning
        self.detection_data = []
        self.publisher_ = self.create_publisher(FiducialMarkerData, 'fiducial_marker_data', 10)
    
    @staticmethod
    def quaternion_to_euler_degrees(quat):
        rpy = [0, 0, 0]
        rpy[0] = math.atan2(2.0 * (quat.w * quat.x + quat.y * quat.z),
                            1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y))
        rpy[1] = math.asin(2.0 * (quat.w * quat.y - quat.z * quat.x))
        rpy[2] = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
        # Convert to degrees
        for i in range(len(rpy)):
            rpy[i] = rpy[i] * 180 / math.pi
        return tuple(rpy)
    
    def tf_callback(self, msg):
        for transform_stamped in msg.transforms:
            if transform_stamped.header.frame_id == CAMERA_FRAME_ID and transform_stamped.child_frame_id.startswith(CHILD_FRAME_ID_PREFIX):
                marker_frame_id = transform_stamped.child_frame_id
                timestamp = transform_stamped.header.stamp.sec + transform_stamped.header.stamp.nanosec * 1e-9
                z_translation = transform_stamped.transform.translation.z
                x_translation = transform_stamped.transform.translation.x
                quaternion = Quaternion(
                    x=transform_stamped.transform.rotation.x,
                    y=transform_stamped.transform.rotation.y,
                    z=transform_stamped.transform.rotation.z,
                    w=transform_stamped.transform.rotation.w,
                )
                marker_yaw = self.quaternion_to_euler_degrees(quaternion)[1]
                self.detection_data.append((timestamp, marker_frame_id, z_translation, marker_yaw))

                # Publish custom message broadcasting docking relevant fiducial marker measurements.
                self.get_logger().info(f"marker_frame_id: {marker_frame_id}, range: {z_translation}, lateral_offset: {x_translation}, yaw: {marker_yaw}")
                # Prepare the data packet
                data_packet = FiducialMarkerData()
                data_packet.marker_frame_id = marker_frame_id
                data_packet.range = z_translation
                data_packet.lateral_offset = x_translation
                data_packet.yaw = marker_yaw

                # Publish the message
                self.publisher_.publish(data_packet)


class FiducialMarkerFVDApp(QMainWindow):
    def __init__(self, data):
        super(FiducialMarkerFVDApp, self).__init__()

        self.data = data

        self.setWindowTitle("Marker Detection Demo")
        self.setGeometry(100, 100, 1200, 600)

        layout = QHBoxLayout()

        # First column for Live Range
        column1 = QVBoxLayout()
        column1_label = QLabel("Live Range")
        column1.addWidget(column1_label)
        self.z_plot_widget = pg.PlotWidget()
        self.z_plot_widget.setLabel("left", "Range", units="m")
        self.z_plot_widget.setLabel("bottom", "Time", units="s")
        self.z_curve = self.z_plot_widget.plot(pen="y")
        column1.addWidget(self.z_plot_widget)
        layout.addLayout(column1)

        # Second column for Live Yaw
        column2 = QVBoxLayout()
        column2_label = QLabel("Live Yaw")
        column2.addWidget(column2_label)
        self.yaw_plot_widget = pg.PlotWidget()
        self.yaw_plot_widget.setLabel("left", "Marker-Yaw", units="degrees")
        self.yaw_plot_widget.setLabel("bottom", "Time", units="s")
        self.yaw_curve = self.yaw_plot_widget.plot(pen="y")
        column2.addWidget(self.yaw_plot_widget)
        layout.addLayout(column2)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(500)  # Update the plot every 500 ms

        self.is_running = True

    def update_plot(self):
        current_time = time.time()
        filtered_data = [
            (t, z, marker_yaw) for t, _, z, marker_yaw in self.data if current_time - t <= 10  # assuming a 10-second window
        ]

        if filtered_data:
            t, z, yaw = zip(*filtered_data)

            # Update the plots
            self.z_curve.setData(t, z)
            self.yaw_curve.setData(t, yaw)

            # Autoscale Y-axes
            self.z_plot_widget.setYRange(min(z), max(z), padding=0.5)
            self.yaw_plot_widget.setYRange(min(yaw), max(yaw), padding=3)
        else:
            self.z_curve.clear()
            self.yaw_curve.clear()
            self.z_plot_widget.setYRange(0, 1, padding=0.1)  # Reset to default range if no data
            self.yaw_plot_widget.setYRange(0, 1, padding=0.1)  # Reset to default range if no data

    def quit(self):
        self.is_running = False


def main(args=None):
    rclpy.init(args=args)
    fiducial_marker_fvd_node = FiducialMarkerFVD()

    if ENABLE_GUI:
        app = QApplication(sys.argv)
        fiducial_marker_fvd_app = FiducialMarkerFVDApp(
            fiducial_marker_fvd_node.detection_data
        )
        fiducial_marker_fvd_app.show()
        app.aboutToQuit.connect(fiducial_marker_fvd_app.quit)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(fiducial_marker_fvd_node)

    try:
        if ENABLE_GUI:
            while fiducial_marker_fvd_app.is_running:
                executor.spin_once(timeout_sec=0.01)
                app.processEvents()
        else:
            rclpy.spin(fiducial_marker_fvd_node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        fiducial_marker_fvd_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()