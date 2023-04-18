import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rosbag2_py import SequentialReader
from sensor_msgs.msg import Image

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

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <ros2_bag_file> <image_topic> <output_video>")
        sys.exit(1)

    bag_file = sys.argv[1]
    image_topic = sys.argv[2]
    output_video = sys.argv[3]

    create_video_from_rosbag(bag_file, image_topic, output_video)
