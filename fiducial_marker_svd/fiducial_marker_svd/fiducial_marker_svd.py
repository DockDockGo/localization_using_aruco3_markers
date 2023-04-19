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

CAMERA_FRAME_ID = "left_hand_camera"
CHILD_FRAME_ID = "marker_0"
MAX_PERMISSIBLE_RANGE_ERROR_M = 0.03
MAX_PERMISSIBLE_YAW_ERROR_DEGREES = 3

GROUND_TRUTH_RANGE = [
    1.4,
    1.439,
    2
]

GROUND_TRUTH_YAW = [
    0,
    10,
    20,
    30,
    40,
]

class FiducialMarkerSVD(Node):
    def __init__(self):
        super().__init__("fiducial_marker_svd_node")
        self.subscription = self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)
        self.subscription  # prevent unused variable warning
        self.detection_data = []

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
                self.detection_data.append((timestamp, z_translation))

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
            if transform_stamped.header.frame_id == CAMERA_FRAME_ID and transform_stamped.child_frame_id == CHILD_FRAME_ID:
                timestamp = transform_stamped.header.stamp.sec + transform_stamped.header.stamp.nanosec * 1e-9
                z_translation = transform_stamped.transform.translation.z
                quaternion = Quaternion(
                    x=transform_stamped.transform.rotation.x,
                    y=transform_stamped.transform.rotation.y,
                    z=transform_stamped.transform.rotation.z,
                    w=transform_stamped.transform.rotation.w,
                )
                marker_yaw = self.quaternion_to_euler_degrees(quaternion)[1]
                self.detection_data.append((timestamp, z_translation, marker_yaw))



class FiducialMarkerSVDApp(QMainWindow):
    def __init__(self, data):
        super(FiducialMarkerSVDApp, self).__init__()

        self.data = data
        self.saved_data = []

        self.setWindowTitle("Marker Detection Demo")
        self.setGeometry(100, 100, 1600, 600)

        outer_layout = QVBoxLayout()
        layout = QHBoxLayout()

        # First column
        column1 = QVBoxLayout()
        column1_label = QLabel()
        column1_label.setText("Live Range Error")
        column1.addWidget(column1_label)
        self.z_error_plot_widget = pg.PlotWidget()
        self.z_error_plot_widget.setLabel("left", "Range-Error", units="m")
        self.z_error_plot_widget.setLabel("bottom", "Time", units="s")
        self.z_error_plot_widget.setYRange(0, 0.005)
        self.z_error_curve = self.z_error_plot_widget.plot(pen="y")
        column1.addWidget(self.z_error_plot_widget)

        self.init_z_labels_and_inputs(column1)

        # Second column
        column2 = QVBoxLayout()
        column2_label = QLabel()
        column2_label.setText("Live Yaw Error")
        column2.addWidget(column2_label)
        self.yaw_error_plot_widget = pg.PlotWidget()
        self.yaw_error_plot_widget.setLabel("left", "Marker-Yaw-Error", units="degrees")
        self.yaw_error_plot_widget.setLabel("bottom", "Time", units="s")
        self.yaw_error_plot_widget.setYRange(0, 5)
        self.yaw_error_curve = self.yaw_error_plot_widget.plot(pen="y")
        column2.addWidget(self.yaw_error_plot_widget)

        self.init_yaw_labels_and_inputs(column2)

        # Sliding window global setting
        self.sliding_window_size_input_layout = QHBoxLayout()
        self.sliding_window_size_label = QLabel("Duration of sliding window (seconds):")
        self.sliding_window_size_input = QLineEdit()
        self.sliding_window_size_input.setText("10")
        self.sliding_window_size_input.setValidator(QDoubleValidator())
        self.sliding_window_size_input.setPlaceholderText(
            "Enter sliding window length in seconds."
        )
        self.sliding_window_size_input_layout.addWidget(self.sliding_window_size_label)
        self.sliding_window_size_input_layout.addWidget(self.sliding_window_size_input)

        outer_layout.addLayout(self.sliding_window_size_input_layout)
        layout.addLayout(column1)
        layout.addLayout(column2)

        # Third column
        column3 = QVBoxLayout()
        column3_label = QLabel()
        column3_label.setText("Validation of saved data points")
        column3.addWidget(column3_label)
        self.error_plot_widget = pg.PlotWidget()
        self.error_plot_widget.setLabel("left", "Range-Error", units="m")
        self.error_plot_widget.setLabel("bottom", "Yaw-Error", units="degrees")
        # self.error_plot_widget.setYRange(0, 0.05)
        # self.error_plot_widget.setXRange(0, 5)
        self.error_curve = self.error_plot_widget.plot(pen="y")
        column3.addWidget(self.error_plot_widget)

        self.max_range_error_label = QLabel()
        column3.addWidget(self.max_range_error_label)

        self.max_yaw_error_label = QLabel()
        column3.addWidget(self.max_yaw_error_label)

        self.pass_fail_label = QLabel()
        self.pass_fail_label.setAlignment(Qt.AlignCenter)
        self.pass_fail_label.setMinimumHeight(50)
        column3.addWidget(self.pass_fail_label)

        layout.addLayout(column3)

        outer_layout.addLayout(layout)

        save_button = QPushButton("Save Latest Data")
        save_button.setStyleSheet("font-size: 18px; height: 50px;")
        save_button.clicked.connect(self.save_latest_data)
        outer_layout.addWidget(save_button)

        reset_saved_data_button = QPushButton("Reset Saved Data")
        reset_saved_data_button.setStyleSheet("font-size: 18px; height: 50px;")
        reset_saved_data_button.clicked.connect(self.reset_saved_data)
        outer_layout.addWidget(reset_saved_data_button)

        central_widget = QWidget()
        central_widget.setLayout(outer_layout)
        self.setCentralWidget(central_widget)

        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # Update the plot every 100 ms

        self.is_running = True

    def init_z_labels_and_inputs(self, layout):
        self.z_label = QLabel()
        layout.addWidget(self.z_label)

        self.z_moving_average_label = QLabel()
        layout.addWidget(self.z_moving_average_label)

        self.z_std_dev_label = QLabel()
        layout.addWidget(self.z_std_dev_label)

        self.snap_gt_label = QLabel()
        self.snap_gt_label.setText("Reset ground truth range to:")
        layout.addWidget(self.snap_gt_label)

        # Reset ground truth range buttons
        buttons_layout = QHBoxLayout()
        for value in GROUND_TRUTH_RANGE:
            button = QPushButton(f"{value}m")
            button.clicked.connect(lambda _, v=value: self.reset_ground_truth_z(v))
            buttons_layout.addWidget(button)
        layout.addLayout(buttons_layout)

        # Ground truth input with label
        ground_truth_input_layout = QHBoxLayout()
        ground_truth_label = QLabel("Ground truth range:")
        self.ground_truth_z_input = QLineEdit()
        self.ground_truth_z_input.setText("1.439")
        self.ground_truth_z_input.setValidator(QDoubleValidator())
        self.ground_truth_z_input.setPlaceholderText("Enter ground truth range")
        ground_truth_input_layout.addWidget(ground_truth_label)
        ground_truth_input_layout.addWidget(self.ground_truth_z_input)
        layout.addLayout(ground_truth_input_layout)

    def reset_ground_truth_z(self, value):
        self.ground_truth_z_input.setText(str(value))

    def init_yaw_labels_and_inputs(self, layout):
        self.yaw_label = QLabel()
        layout.addWidget(self.yaw_label)

        self.yaw_moving_average_label = QLabel()
        layout.addWidget(self.yaw_moving_average_label)

        self.yaw_std_dev_label = QLabel()
        layout.addWidget(self.yaw_std_dev_label)

        self.snap_yaw_gt_label = QLabel()
        self.snap_yaw_gt_label.setText("Reset ground truth yaw (in degrees) to:")
        layout.addWidget(self.snap_yaw_gt_label)

        # Reset ground truth range buttons
        buttons_layout = QHBoxLayout()
        for value in GROUND_TRUTH_YAW:
            button = QPushButton(f"{value} deg")
            button.clicked.connect(lambda _, v=value: self.reset_ground_truth_yaw(v))
            buttons_layout.addWidget(button)
        layout.addLayout(buttons_layout)

        # Ground truth yaw input with label
        ground_truth_yaw_input_layout = QHBoxLayout()
        ground_truth_yaw_label = QLabel("Ground truth yaw:")
        self.ground_truth_yaw_input = QLineEdit()
        self.ground_truth_yaw_input.setText("10")
        self.ground_truth_yaw_input.setValidator(QDoubleValidator())
        self.ground_truth_yaw_input.setPlaceholderText("Enter ground truth marker yaw.")
        ground_truth_yaw_input_layout.addWidget(ground_truth_yaw_label)
        ground_truth_yaw_input_layout.addWidget(self.ground_truth_yaw_input)
        layout.addLayout(ground_truth_yaw_input_layout)

    def reset_ground_truth_yaw(self, value):
        self.ground_truth_yaw_input.setText(str(value))

    def save_latest_data(self):
        if self.data:
            t, range, yaw = self.data[-1]
            range_error = np.abs(range - np.double(self.ground_truth_z_input.text()))
            yaw_error = np.abs(yaw - np.double(self.ground_truth_yaw_input.text()))
            latest_data = t, range, range_error, yaw, yaw_error
            self.saved_data.append(latest_data)
            print(f"Saved data: {latest_data}")

    def reset_saved_data(self):
        self.saved_data = []

    def update_plot(self):
        current_time = time.time()
        sliding_time_window_str = (
            "10"
            if not bool(self.sliding_window_size_input.text())
            else self.sliding_window_size_input.text()
        )
        filtered_data = [
            (t, z, marker_yaw) for t, z, marker_yaw in self.data if current_time - t <= np.double(sliding_time_window_str)
        ]

        if filtered_data:
            t, z, yaw = zip(*filtered_data)
            
            self.z_error_curve.setData(t, np.abs(z - np.double(self.ground_truth_z_input.text())))
            self.z_label.setText(f"Latest range: {z[-1]:.3f}")
            self.z_moving_average_label.setText(f"Moving average of range: {np.average(z):.3f}")
            self.z_std_dev_label.setText(f"Standard deviation on range: {np.std(z):.3f}")

            self.yaw_error_curve.setData(t, np.abs(yaw - np.double(self.ground_truth_yaw_input.text())))
            self.yaw_label.setText(f"Latest yaw: {yaw[-1]:.3f}")
            self.yaw_moving_average_label.setText(f"Moving average of yaw: {np.average(yaw):.3f}")
            self.yaw_std_dev_label.setText(f"Standard deviation on yaw: {np.std(yaw):.3f}")

        else:
            self.z_error_curve.clear()
            self.z_label.setText("No Data")
            self.yaw_error_curve.clear()
            self.yaw_label.setText("No Data")
        
        if self.saved_data:
            t, range, range_error, yaw, yaw_error = zip(*self.saved_data)
            self.error_curve.setData(yaw_error, range_error, symbol='o', pen=None)

            max_range_error = max(range_error)
            max_yaw_error = max(yaw_error)

            self.max_range_error_label.setText(f"Max range error: {max_range_error:.3f}")
            self.max_yaw_error_label.setText(f"Max yaw error: {max_yaw_error:.3f}")

            if max_range_error < MAX_PERMISSIBLE_RANGE_ERROR_M and max_yaw_error < MAX_PERMISSIBLE_YAW_ERROR_DEGREES:
                self.pass_fail_label.setStyleSheet("background-color: green;")
                self.pass_fail_label.setText("PASS")
            else:
                self.pass_fail_label.setStyleSheet("background-color: red;")
                self.pass_fail_label.setText("FAIL")
        else:
            self.error_curve.clear()
            self.max_range_error_label.setText("No Data")
            self.max_yaw_error_label.setText("No Data")
            self.pass_fail_label.clear()
            self.pass_fail_label.setStyleSheet("")

        if self.saved_data:
            t, range, range_error, yaw, yaw_error = zip(*self.saved_data)

            # Calculate colors based on range_error and yaw_error values
            colors = [
                pg.mkColor("g") if (re < 0.03 and ye < 3) else pg.mkColor("r")
                for re, ye in zip(range_error, yaw_error)
            ]

            # Create a ScatterPlotItem with the updated colors
            scatter_plot_item = pg.ScatterPlotItem(x=yaw_error, y=range_error, pen=colors)

            # Update the scatter plot
            self.error_plot_widget.clear()
            self.error_plot_widget.addItem(scatter_plot_item)

            # Update the maximum error labels
            self.max_range_error_label.setText(f"Max Range Error: {max(range_error) * 1000:.3f} mm")
            self.max_yaw_error_label.setText(f"Max Yaw Error: {max(yaw_error):.3f}°")

            if max(range_error) < 0.03 and max(yaw_error) < 3:
                self.pass_fail_label.setStyleSheet("background-color: green")
                self.pass_fail_label.setText("PASS")
            else:
                self.pass_fail_label.setStyleSheet("background-color: red")
                self.pass_fail_label.setText("FAIL")
        else:
            self.error_plot_widget.clear()


    def quit(self):
        self.is_running = False


def main(args=None):
    rclpy.init(args=args)
    fiducial_marker_svd_node = FiducialMarkerSVD()

    app = QApplication(sys.argv)
    fiducial_marker_svd_app = FiducialMarkerSVDApp(
        fiducial_marker_svd_node.detection_data
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
