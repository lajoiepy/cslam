#!/usr/bin/env python
import rosbag
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
bridge = CvBridge()

bag_name = '/home/lajoiepy/Documents/datasets/Poly4eCorridorsRealsense/18jan/Train1_decompressed.bag'

with rosbag.Bag(bag_name[:-4] + '_fixed.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag(bag_name).read_messages():
        if topic == "/camera/aligned_depth_to_color/image_raw":
            #msg.step = msg.width * 2
            msg.encoding = 'mono8'
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
            msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
            print(msg.encoding)

        outbag.write(topic, msg, t)