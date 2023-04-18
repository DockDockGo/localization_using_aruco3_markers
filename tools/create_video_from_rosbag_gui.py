import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rosbag2_py import SequentialReader
from sensor_msgs.msg import Image
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QRadioButton, QFileDialog, QMessageBox
from PyQt5.QtCore import Qt

def create_video_from_rosbag(bag_file, image_topic, output_video):
    rclpy.init()

    bridge = CvBridge()
    reader = SequentialReader()
    reader.open(bag_file)
    storage_options, converter_options = reader.get_all_options()
    topics = reader.get_all_topics_and_types()

    video_writer = None
    frame_size = None

    while reader.has_next():
        topic, msg, timestamp = reader.read_next()

        if topic == image_topic and isinstance(msg, Image):
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except ValueError:
                continue

            if video_writer is None:
                frame_size = (cv_image.shape[1], cv_image.shape[0])
                video_writer = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'XVID'), 30, frame_size)

            video_writer.write(cv_image)

    if video_writer is not None:
        video_writer.release()

    reader.close()
    rclpy.shutdown()

class RosbagToVideoGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        self.setWindowTitle('Rosbag to Video Converter')

        vbox = QVBoxLayout()

        hbox1 = QHBoxLayout()
        self.bag_file_input = QLineEdit(self)
        self.bag_file_input.setPlaceholderText("Select a ROS bag file")
        self.bag_file_input.setReadOnly(True)
        self.bag_file_input.clicked = lambda: self.select_bag_file()
        self.bag_file_input.mousePressEvent = lambda event: self.bag_file_input.clicked()

        hbox1.addWidget(QLabel("ROS bag file:"))
        hbox1.addWidget(self.bag_file_input)

        vbox.addLayout(hbox1)

        vbox.addWidget(QLabel("Available topics:"))

        self.topic_layout = QVBoxLayout()
        vbox.addLayout(self.topic_layout)

        hbox2 = QHBoxLayout()
        self.output_video_input = QLineEdit(self)
        self.output_video_input.setPlaceholderText("Enter output video file name/path")

        hbox2.addWidget(QLabel("Output video file:"))
        hbox2.addWidget(self.output_video_input)

        vbox.addLayout(hbox2)

        self.convert_button = QPushButton('Convert', self)
        self.convert_button.clicked.connect(self.convert_rosbag_to_video)
        vbox.addWidget(self.convert_button)

        self.setLayout(vbox)

    def select_bag_file(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ReadOnly
        bag_file, _ = QFileDialog.getOpenFileName(self, "Select ROS bag file", "", "ROS bag files (*.db3);;All Files (*)", options=options)
        if bag_file:
            self.bag_file_input.setText(bag_file)
            self.populate_topics()

    def populate_topics(self):
        for i in reversed(range(self.topic_layout.count())):
            self.topic_layout.itemAt(i).widget().setParent(None)

        topics = self.get_available_topics()

        for topic in topics:
            radio_button = QRadioButton(topic, self)
            self.topic_layout.addWidget(radio_button)

    def get_available_topics(self):
        bag_file = self.bag_file_input.text()
        if not bag_file:
            return []

        rclpy.init()

        reader = SequentialReader()
        reader.open(bag_file)
        storage_options, converter_options = reader.get_all_options()
        topics_info = reader.get_all_topics_and_types()

        rclpy.shutdown()

        image_topics = []
        for topic, topic_type in topics_info.items():
            if topic_type == 'sensor_msgs/msg/Image':
                image_topics.append(topic)

        return image_topics

    def convert_rosbag_to_video(self):
        bag_file = self.bag_file_input.text()
        output_video = self.output_video_input.text()

        if not bag_file or not output_video:
            QMessageBox.warning(self, "Warning", "Please select a ROS bag file and enter an output video file name/path.")
            return

        selected_topic = None
        for i in range(self.topic_layout.count()):
            radio_button = self.topic_layout.itemAt(i).widget()
            if radio_button.isChecked():
                selected_topic = radio_button.text()
                break

        if not selected_topic:
            QMessageBox.warning(self, "Warning", "Please select a topic.")
            return

        create_video_from_rosbag(bag_file, selected_topic, output_video)

        QMessageBox.information(self, "Success", "The ROS bag file has been successfully converted to a video.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RosbagToVideoGUI()
    window.show()
    sys.exit(app.exec_())

