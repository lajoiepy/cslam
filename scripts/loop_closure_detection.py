#!/usr/bin/env python

# Loop Closure Detection service
# Multiple implementations of loop closure detection for benchmarking

import rclpy
from sensor_msgs.msg import Image

from cslam_interfaces.srv import DetectLoopClosure
from external_loop_closure_detection.netvlad_loop_closure_detection import NetVLADLoopClosureDetection
from external_loop_closure_detection.vit_loop_closure_detection import ViTLoopClosureDetection

class LoopClosureDetection(object):

    def init(self):
        self.node = rclpy.create_node('loop_closure_detection')
        self.node.declare_parameters(
            namespace='',
            parameters=[
                ('threshold', None),
                ('min_inbetween_keyframes', None),
                ('nb_best_matches', None),
                ('technique', None),
                ('pca', None),
                ('resume', None),
                ('checkpoint', None),
                ('crop_size', None)
            ]
        )
        params = {}
        params['threshold'] = self.node.get_parameter('threshold').value
        params['min_inbetween_keyframes'] = self.node.get_parameter('min_inbetween_keyframes').value
        params['nb_best_matches'] = self.node.get_parameter('nb_best_matches').value
        params['technique'] = self.node.get_parameter('technique').value
        if params['technique'].lower() == 'netvlad':
            params['pca'] = self.node.get_parameter('pca').value
        params['resume'] = self.node.get_parameter('resume').value
        params['checkpoint'] = self.node.get_parameter('checkpoint').value
        params['crop_size'] = self.node.get_parameter('crop_size').value

        if params['technique'].lower() == 'netvlad':
            self.lcd = NetVLADLoopClosureDetection(params)
        elif params['technique'].lower() == 'vit':
            self.lcd = ViTLoopClosureDetection(params)
        else:
            self.node.get_logger().err('ERROR: Unknown technique')

        self.srv = self.node.create_service(DetectLoopClosure, 'detect_loop_closure', self.service)

        rclpy.spin(self.node)

    def service(self, req):
        # Call all methods we want to test
        res = self.lcd.detect_loop_closure_service(req)
        return res

if __name__ == '__main__':
    
    rclpy.init(args=None)
    lcd = LoopClosureDetection()
    lcd.init()
