#!/usr/bin/env python3

# Loop Closure Detection service
# Multiple implementations of loop closure detection for benchmarking

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cslam_loop_detection.srv import DetectLoopClosure
from loop_closure_detection.global_image_descriptor_loop_closure_detection import GlobalImageDescriptorLoopClosureDetection

from example_interfaces.srv import AddTwoInts


class LoopClosureDetection(Node):

    def __init__(self):
        super().__init__('loop_closure_detection')

        self.declare_parameters(namespace='',
                                parameters=[('threshold', None),
                                            ('min_inbetween_keyframes', None),
                                            ('nb_best_matches', None),
                                            ('technique', None), ('pca', None),
                                            ('resume', None),
                                            ('checkpoint', None),
                                            ('crop_size', None),
                                            ('robot_id', None),
                                            ('global_descriptor_topic', None)])
        params = {}
        params['threshold'] = self.get_parameter('threshold').value
        params['min_inbetween_keyframes'] = self.get_parameter(
            'min_inbetween_keyframes').value
        params['nb_best_matches'] = self.get_parameter('nb_best_matches').value
        params['technique'] = self.get_parameter('technique').value
        params['resume'] = self.get_parameter('resume').value
        params['checkpoint'] = self.get_parameter('checkpoint').value
        params['crop_size'] = self.get_parameter('crop_size').value
        params['robot_id'] = self.get_parameter('robot_id').value

        self.glcd = GlobalImageDescriptorLoopClosureDetection(params, self)
        self.srv = self.create_service(DetectLoopClosure,
                                       'detect_loop_closure', self.service)

    def service(self, req, res):
        # Call all methods we want to test
        res = self.glcd.detect_loop_closure_service(req, res)
        res.from_id = req.image.id
        return res


if __name__ == '__main__':

    rclpy.init(args=None)
    lcd = LoopClosureDetection()
    lcd.get_logger().info('Initialization done.')
    rclpy.spin(lcd)
    rclpy.shutdown()
