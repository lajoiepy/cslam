#!/usr/bin/env python

# Loop Closure Detection service
# Multiple implementations of loop closure detection for benchmarking

import rospy
from sensor_msgs.msg import Image

from external_loop_closure_detection.srv import DetectLoopClosure, DetectLoopClosureResponse
from external_loop_closure_detection.netvlad_loop_closure_detection import NetVLADLoopClosureDetection

class LoopClosureDetection(object):

    def init(self):
        rospy.init_node('loop_closure_detection', anonymous=True)

        params = [] # TODO: Parse params

        self.netvlad = NetVLADLoopClosureDetection(params)

        service = rospy.Service('detect_loop_closure', DetectLoopClosure, self.service)

        rospy.spin()

    def service(self, req):
        # Call all methods we want to test
        res = self.netvlad.detect_loop_closure_service(req)
        return res

if __name__ == '__main__':

    try:
        lcd = LoopClosureDetection()
        lcd.init()
    except rospy.ROSInterruptException:
        pass
