#!/usr/bin/env python
import rosbag
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
bridge = CvBridge()

bag_name = '/home/lajoiepy/Documents/datasets/Poly4eCorridorsRealsense/16Feb2022/Train1.bag'

with rosbag.Bag(bag_name[:-4] + '_fixed.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag(bag_name).read_messages():
        if topic == "/camera/aligned_depth_to_color/image_raw/compressedDepth":
            msg.format = msg.format + " png"
        outbag.write(topic, msg, t)