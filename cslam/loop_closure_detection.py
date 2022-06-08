#!/usr/bin/env python3

# Loop Closure Detection service
# Abstraction to support multiple implementations of loop closure detection for benchmarking

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cslam_loop_detection.srv import DetectLoopClosure
from cslam.global_image_descriptor_loop_closure_detection import GlobalImageDescriptorLoopClosureDetection

from example_interfaces.srv import AddTwoInts


class LoopClosureDetection(Node):
    """ Global image descriptor matching for loop closure detection """

    def __init__(self):
        """Initialization and parameter parsing"""
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
                                            ('nb_robots', None),
                                            ('similarity_loc', 1.0),
                                            ('similarity_scale', 0.25),
                                            ('loop_closure_budget', 5),
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
        params['nb_robots'] = self.get_parameter('nb_robots').value
        params['similarity_loc'] = self.get_parameter('similarity_loc').value
        params['similarity_scale'] = self.get_parameter('similarity_scale').value
        params['loop_closure_budget'] = self.get_parameter('loop_closure_budget').value

        self.glcd = GlobalImageDescriptorLoopClosureDetection(params, self)
        self.srv = self.create_service(DetectLoopClosure,
                                       'detect_loop_closure', self.service)

    def service(self, req, res):
        """Service callback to detect loop closures associate to the keyframe 

        Args:
            req (cslam_loop_detection::srv::DetectLoopClosure::req): Keyframe data
            res (cslam_loop_detection::srv::DetectLoopClosure::res): Place recognition match data

        Returns:
            cslam_loop_detection::srv::DetectLoopClosure::res: Place recognition match data
        """
        res = self.glcd.detect_loop_closure_service(req, res)
        res.from_id = req.image.id
        return res


if __name__ == '__main__':

    rclpy.init(args=None)
    lcd = LoopClosureDetection()
    lcd.get_logger().info('Initialization done.')
    rclpy.spin(lcd)
    rclpy.shutdown()
