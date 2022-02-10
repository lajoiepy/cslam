#!/usr/bin/env python

# Loop Closure Detection service
# Multiple implementations of loop closure detection for benchmarking

import rospy
from sensor_msgs.msg import Image

from external_loop_closure_detection.srv import DetectLoopClosure, DetectLoopClosureResponse
from external_loop_closure_detection.netvlad_loop_closure_detection import NetVLADLoopClosureDetection
from external_loop_closure_detection.vit_loop_closure_detection import ViTLoopClosureDetection

class LoopClosureDetection(object):

    def init(self):
        rospy.init_node('loop_closure_detection', anonymous=True)

        params = {}
        params['threshold'] = rospy.get_param('~threshold')
        params['min_inbetween_keyframes'] = rospy.get_param('~min_inbetween_keyframes')
        params['nb_best_matches'] = rospy.get_param('~nb_best_matches')
        params['technique'] = rospy.get_param('~technique')
        if params['technique'].lower() == 'netvlad':
            params['pca'] = rospy.get_param('~pca')
        params['resume'] = rospy.get_param('~resume', False)
        params['checkpoint'] = rospy.get_param('~checkpoint', 'none')

        if params['technique'].lower() == 'netvlad':
            self.lcd = NetVLADLoopClosureDetection(params)
        elif params['technique'].lower() == 'vit':
            self.lcd = ViTLoopClosureDetection(params)
        else:
            rospy.logerr('ERROR: Unknown technique')

        self.srv = rospy.Service('detect_loop_closure', DetectLoopClosure, self.service)

        rospy.spin()

    def service(self, req):
        # Call all methods we want to test
        res = self.lcd.detect_loop_closure_service(req)
        return res

if __name__ == '__main__':

    try:
        lcd = LoopClosureDetection()
        lcd.init()
    except rospy.ROSInterruptException:
        pass
