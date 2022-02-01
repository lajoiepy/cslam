#!/usr/bin/env python

# Loop Closure Detection service
# Multiple implementations of loop closure detection for benchmarking

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from external_loop_closure_detection.srv import DetectLoopClosure, DetectLoopClosureResponse

def detect_loop_closure_netvlad(req):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(req.image, desired_encoding='passthrough')

    # TODO: Netvlad processing
    rospy.logwarn("NetVLAD processing")

    return DetectLoopClosureResponse(is_detected=True, detected_loop_closure_id=req.image.header.seq-1)


def main():
    rospy.init_node('loop_closure_detection', anonymous=True)

    service = rospy.Service('detect_loop_closure', DetectLoopClosure, detect_loop_closure_netvlad)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
