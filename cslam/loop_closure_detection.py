#!/usr/bin/env python3

# Loop Closure Detection service
# Abstraction to support multiple implementations of loop closure detection for benchmarking

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cslam.global_image_descriptor_loop_closure_detection import GlobalImageDescriptorLoopClosureDetection

class LoopClosureDetection(Node):
    """ Global image descriptor matching for loop closure detection """

    def __init__(self):
        """Initialization and parameter parsing"""
        super().__init__('loop_closure_detection')

        self.declare_parameters(namespace='',
                                parameters=[
                                    ('similarity_threshold', None), ('global_descriptor_technique', None),
                                    ('pca_checkpoint', None), ('nn_checkpoint', None),
                                    ('robot_id', None), ('nb_robots', None),
                                    ('similarity_loc', 1.0),
                                    ('similarity_scale', 0.25),
                                    ('loop_closure_budget', 5),
                                    ('detection_period', 5),
                                    ('nb_best_matches', 10),
                                    ('image_crop_size', None),
                                    ('intra_loop_min_inbetween_keyframes', 10),
                                    ('intra_robot_loop_closure_detection',
                                     False), ('global_descriptor_topic', None)
                                ])
        params = {}
        params['similarity_threshold'] = self.get_parameter('similarity_threshold').value
        params['intra_loop_min_inbetween_keyframes'] = self.get_parameter(
            'intra_loop_min_inbetween_keyframes').value
        params['nb_best_matches'] = self.get_parameter('nb_best_matches').value
        params['global_descriptor_technique'] = self.get_parameter('global_descriptor_technique').value
        params['nn_checkpoint'] = self.get_parameter('nn_checkpoint').value
        params['robot_id'] = self.get_parameter('robot_id').value
        params['nb_robots'] = self.get_parameter('nb_robots').value
        params['similarity_loc'] = self.get_parameter('similarity_loc').value
        params['similarity_scale'] = self.get_parameter(
            'similarity_scale').value
        params['loop_closure_budget'] = self.get_parameter(
            'loop_closure_budget').value
        params['intra_robot_loop_closure_detection'] = self.get_parameter(
            'intra_robot_loop_closure_detection').value
        params['detection_period'] = self.get_parameter(
            'detection_period').value
        params["image_crop_size"] = self.get_parameter(
            'image_crop_size').value

        self.glcd = GlobalImageDescriptorLoopClosureDetection(params, self)
        self.loop_detection_timers = self.create_timer(
            params['detection_period'], self.glcd.detect_inter)

if __name__ == '__main__':

    rclpy.init(args=None)
    lcd = LoopClosureDetection()
    lcd.get_logger().info('Initialization done.')
    rclpy.spin(lcd)
    rclpy.shutdown()
