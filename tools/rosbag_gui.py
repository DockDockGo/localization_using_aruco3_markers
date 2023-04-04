import sys
import os
import subprocess
import json
from PyQt5.QtCore import QTimer, QTime
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, QScrollArea, QCheckBox, QGridLayout, QFrame

class RosbagRecorder(QWidget):
    def __init__(self):
        super().__init__()

        self.process = None
        self.topic_checkboxes = {}
        self.initUI()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_timer)
        self.time_elapsed = QTime(0, 0)

    def initUI(self):
        self.setWindowTitle('Rosbag Recorder')

        vbox = QVBoxLayout()

        scroll_area = QScrollArea(self)
        scroll_content = QWidget(scroll_area)
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(scroll_content)

        available_topics = self.get_available_topics()
        for topic in available_topics:
            checkbox = QCheckBox(topic, self)
            checkbox.setChecked(True)
            self.topic_checkboxes[topic] = checkbox
            scroll_layout.addWidget(checkbox)

        self.output_path_input = QLineEdit(self)
        self.output_path_input.setPlaceholderText("Enter output file name/path")

        self.gt_range_input = QLineEdit(self)
        self.gt_range_input.setPlaceholderText("Enter gt_range")
        self.gt_viewing_angle_input = QLineEdit(self)
        self.gt_viewing_angle_input.setPlaceholderText("Enter gt_viewing_angle")

        self.record_button = QPushButton('Start Recording', self)
        self.record_button.clicked.connect(self.toggle_recording)

        vbox.addWidget(QLabel("Available Topics:"))
        vbox.addWidget(scroll_area)
        vbox.addWidget(QLabel("Output file name/path:"))
        vbox.addWidget(self.output_path_input)
        vbox.addWidget(QLabel("gt_range:"))
        vbox.addWidget(self.gt_range_input)
        vbox.addWidget(QLabel("gt_viewing_angle:"))
        vbox.addWidget(self.gt_viewing_angle_input)
        vbox.addWidget(self.record_button)

        self.timer_label = QLabel("Recording Time: 00:00:00", self)
        vbox.addWidget(self.timer_label)

        self.setLayout(vbox)

    def update_timer(self):
        self.time_elapsed = self.time_elapsed.addSecs(1)
        self.timer_label.setText(f"Recording Time: {self.time_elapsed.toString()}")


    def toggle_recording(self):
        if self.process is None:
            self.start_recording()
        else:
            self.stop_recording()

    def start_recording(self):
        self.time_elapsed.setHMS(0, 0, 0)
        self.timer_label.setText("Recording Time: 00:00:00")
        self.timer.start(1000)  # Update the timer every 1 second (1000 ms)

        selected_topics = [topic for topic, checkbox in self.topic_checkboxes.items() if checkbox.isChecked()]

        if not selected_topics:
            print("No topics selected.")
            return

        output_path = self.output_path_input.text().strip()
        if not output_path:
            output_path = "my_rosbag"

        topics = ' '.join(selected_topics)
        cmd = f"ros2 bag record -o {output_path} {topics}"
        self.process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        metadata = {
            'topics': selected_topics,
            'output_file': f"{output_path}.db3",
            'additional_information': 'You can add any custom metadata here',
            'gt_range': self.gt_range_input.text(),
            'gt_viewing_angle': self.gt_viewing_angle_input.text()
        }

        with open(f"{output_path}_metadata.json", 'w') as metadata_file:
            json.dump(metadata, metadata_file, indent=2)

        self.record_button.setText("Stop Recording")

    def stop_recording(self):
        if self.process is not None:
            self.process.terminate()
            self.process.wait()
            self.process = None
            self.timer.stop()

        self.record_button.setText("Start Recording")

    def get_available_topics(self):
        cmd = "ros2 topic list"
        process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, _ = process.communicate()
        topics = stdout.decode('utf-8').split('\n')
        topics = [topic.strip() for topic in topics if topic.strip()]
        return topics

if __name__ == '__main__':
    app = QApplication(sys.argv)
    recorder = RosbagRecorder()
    recorder.show()
    sys.exit(app.exec_())
