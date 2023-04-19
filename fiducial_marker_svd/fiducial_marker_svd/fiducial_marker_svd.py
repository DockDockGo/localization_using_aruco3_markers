import sys
import time

import numpy as np
import pyqtgraph as pg
import rclpy
from PyQt5.QtWidgets import QApplication, QLabel, QLineEdit, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtGui import QDoubleValidator


from rclpy.node import Node
from tf2_msgs.msg import TFMessage

CAMERA_FRAME_ID = "left_hand_camera"
CHILD_FRAME_ID = "marker_0"


class FiducialMarkerSVD(Node):
    def __init__(self):
        super().__init__("fiducial_marker_svd_node")
        self.subscription = self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)
        self.subscription  # prevent unused variable warning
        self.left_hand_camera_marker_0 = []

    def tf_callback(self, msg):
        for transform_stamped in msg.transforms:
            if (
                transform_stamped.header.frame_id == CAMERA_FRAME_ID
                and transform_stamped.child_frame_id == CHILD_FRAME_ID
            ):
                timestamp = (
                    transform_stamped.header.stamp.sec
                    + transform_stamped.header.stamp.nanosec * 1e-9
                )
                z_translation = transform_stamped.transform.translation.z
                self.left_hand_camera_marker_0.append((timestamp, z_translation))


class FiducialMarkerSVDApp(QMainWindow):
    def __init__(self, data):
        super(FiducialMarkerSVDApp, self).__init__()

        self.data = data
        self.setWindowTitle("Range Error vs Time")
        self.setGeometry(100, 100, 800, 600)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel("left", "Range-Error", units="m")
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.setYRange(0, 0.005)
        # self.plot_widget.autoRange(padding = 0.5)
        self.curve = self.plot_widget.plot(pen="y")

        layout = QVBoxLayout()
        layout.addWidget(self.plot_widget)

        self.z_label = QLabel()
        layout.addWidget(self.z_label)

        self.z_moving_average_label = QLabel()
        layout.addWidget(self.z_moving_average_label)

        self.z_std_dev_label = QLabel()
        layout.addWidget(self.z_std_dev_label)

        self.ground_truth_z_input = QLineEdit()
        self.ground_truth_z_input.setText("1.439")
        self.ground_truth_z_input.setValidator(QDoubleValidator())
        self.ground_truth_z_input.setPlaceholderText("Enter ground truth range")
        layout.addWidget(self.ground_truth_z_input)

        self.z_sliding_window_size_input = QLineEdit()
        self.z_sliding_window_size_input.setText("10")
        self.z_sliding_window_size_input.setValidator(QDoubleValidator())
        self.z_sliding_window_size_input.setPlaceholderText(
            "Enter sliding window length in seconds."
        )
        layout.addWidget(self.z_sliding_window_size_input)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # Update the plot every 100 ms

        self.is_running = True

    def update_plot(self):
        current_time = time.time()
        sliding_time_window_str = (
            "10"
            if not bool(self.z_sliding_window_size_input.text())
            else self.z_sliding_window_size_input.text()
        )
        filtered_data = [
            (t, z) for t, z in self.data if current_time - t <= np.double(sliding_time_window_str)
        ]

        if filtered_data:
            t, z = zip(*filtered_data)
            self.curve.setData(t, np.abs(z - np.double(self.ground_truth_z_input.text())))
            self.z_label.setText(f"Latest Z: {z[-1]:.3f}")
            self.z_moving_average_label.setText(f"Moving average of Z: {np.average(z):.3f}")
            self.z_std_dev_label.setText(f"Standard deviation on Z window: {np.std(z):.3f}")
        else:
            self.curve.clear()
            self.z_label.setText("No Data")

    def quit(self):
        self.is_running = False


def main(args=None):
    rclpy.init(args=args)
    fiducial_marker_svd_node = FiducialMarkerSVD()

    app = QApplication(sys.argv)
    fiducial_marker_svd_app = FiducialMarkerSVDApp(
        fiducial_marker_svd_node.left_hand_camera_marker_0
    )
    fiducial_marker_svd_app.show()

    app.aboutToQuit.connect(fiducial_marker_svd_app.quit)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(fiducial_marker_svd_node)

    try:
        while fiducial_marker_svd_app.is_running:
            executor.spin_once(timeout_sec=0.01)
            app.processEvents()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        fiducial_marker_svd_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
