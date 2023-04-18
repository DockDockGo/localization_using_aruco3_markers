from pathlib import Path

from rosbags.highlevel import AnyReader

# create reader instance and open for reading
with AnyReader([Path('/home/sid/mrsd_project/ros2_ws/src/tools/my_rosbag/my_rosbag_0.db3')]) as reader:
    connections = [x for x in reader.connections if x.topic == '/imu_raw/Imu']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
         msg = reader.deserialize(rawdata, connection.msgtype)
         print(msg.header.frame_id)