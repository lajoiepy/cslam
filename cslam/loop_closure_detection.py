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

        self.declare_parameters(
            namespace='',
            parameters=[('similarity_threshold', None),
                        ('global_descriptor_technique', None),
                        ('pca_checkpoint', None), ('nn_checkpoint', None),
                        ('robot_id', None), ('nb_robots', None),
                        ('similarity_loc', 1.0), ('similarity_scale', 0.25),
                        ('loop_closure_budget', 5), ('detection_period', 5),
                        ('nb_best_matches', 10), ('image_crop_size', None),
                        ('intra_loop_min_inbetween_keyframes', 10),
                        ('max_heartbeat_delay_sec', 5),
                        ('heartbeat_period_sec', 0.5),
                        ('global_descriptor_publication_period_sec', 1.0),
                        ('global_descriptor_publication_max_elems_per_msg', 10),
                        ('enable_neighbor_monitoring', False),
                        ('intra_robot_loop_closure_detection', False),
                        ('global_descriptor_topic', None)])
        self.params = {}
        self.params['similarity_threshold'] = self.get_parameter(
            'similarity_threshold').value
        self.params['intra_loop_min_inbetween_keyframes'] = self.get_parameter(
            'intra_loop_min_inbetween_keyframes').value
        self.params['nb_best_matches'] = self.get_parameter('nb_best_matches').value
        self.params['global_descriptor_technique'] = self.get_parameter(
            'global_descriptor_technique').value
        self.params['nn_checkpoint'] = self.get_parameter('nn_checkpoint').value
        self.params['robot_id'] = self.get_parameter('robot_id').value
        self.params['nb_robots'] = self.get_parameter('nb_robots').value
        self.params['similarity_loc'] = self.get_parameter('similarity_loc').value
        self.params['similarity_scale'] = self.get_parameter(
            'similarity_scale').value
        self.params['loop_closure_budget'] = self.get_parameter(
            'loop_closure_budget').value
        self.params['intra_robot_loop_closure_detection'] = self.get_parameter(
            'intra_robot_loop_closure_detection').value
        self.params['detection_period'] = self.get_parameter(
            'detection_period').value
        self.params["image_crop_size"] = self.get_parameter('image_crop_size').value
        self.params["enable_neighbor_monitoring"] = self.get_parameter('enable_neighbor_monitoring').value
        self.params["max_heartbeat_delay_sec"] = self.get_parameter('max_heartbeat_delay_sec').value
        self.params["heartbeat_period_sec"] = self.get_parameter('heartbeat_period_sec').value
        self.params["global_descriptor_publication_period_sec"] = self.get_parameter('global_descriptor_publication_period_sec').value
        self.params["global_descriptor_publication_max_elems_per_msg"] = self.get_parameter('global_descriptor_publication_max_elems_per_msg').value

        self.glcd = GlobalImageDescriptorLoopClosureDetection(self.params, self)
        self.loop_detection_timer = self.create_timer(
            self.params['detection_period'], self.glcd.detect_inter)

if __name__ == '__main__':

    rclpy.init(args=None)
    lcd = LoopClosureDetection()
    lcd.get_logger().info('Initialization done.')
    rclpy.spin(lcd)
    rclpy.shutdown()
