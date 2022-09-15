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
            parameters=[('frontend.similarity_threshold', None),
                        ('frontend.global_descriptor_technique', None),
                        ('frontend.netvlad.pca_checkpoint', None), ('frontend.nn_checkpoint', None),
                        ('robot_id', None), ('nb_robots', None),
                        ('frontend.inter_robot_loop_closure_budget', 5),
                        ('frontend.inter_robot_detection_period_sec', 5),
                        ('nb_best_matches', 10), ('frontend.image_crop_size', None),
                        ('frontend.intra_loop_min_inbetween_keyframes', 10),
                        ('neighbor_management.max_heartbeat_delay_sec', 5),
                        ('neighbor_management.heartbeat_period_sec', 0.5),
                        ('frontend.global_descriptor_publication_period_sec', 1.0),
                        ('frontend.global_descriptor_publication_max_elems_per_msg',
                         10), ('neighbor_management.enable_neighbor_monitoring', False),
                        ('frontend.enable_intra_robot_loop_closures', False),
                        ('frontend.global_descriptor_topic', None),
                        ('frontend.use_vertex_cover_selection', True),
                        ('frontend.cosplace.descriptor_dim', 512),
                        ('frontend.cosplace.backbone', "resnet101"),])
        self.params = {}
        self.params['frontend.similarity_threshold'] = self.get_parameter(
            'frontend.similarity_threshold').value
        self.params['frontend.intra_loop_min_inbetween_keyframes'] = self.get_parameter(
            'frontend.intra_loop_min_inbetween_keyframes').value
        self.params['nb_best_matches'] = self.get_parameter(
            'nb_best_matches').value
        self.params['frontend.global_descriptor_technique'] = self.get_parameter(
            'frontend.global_descriptor_technique').value
        self.params['frontend.nn_checkpoint'] = self.get_parameter(
            'frontend.nn_checkpoint').value
        self.params['robot_id'] = self.get_parameter('robot_id').value
        self.params['nb_robots'] = self.get_parameter('nb_robots').value
        self.params['frontend.inter_robot_loop_closure_budget'] = self.get_parameter(
            'frontend.inter_robot_loop_closure_budget').value
        self.params['frontend.enable_intra_robot_loop_closures'] = self.get_parameter(
            'frontend.enable_intra_robot_loop_closures').value
        self.params['frontend.inter_robot_detection_period_sec'] = self.get_parameter(
            'frontend.inter_robot_detection_period_sec').value
        self.params["frontend.image_crop_size"] = self.get_parameter(
            'frontend.image_crop_size').value
        self.params["neighbor_management.enable_neighbor_monitoring"] = self.get_parameter(
            'neighbor_management.enable_neighbor_monitoring').value
        self.params["neighbor_management.max_heartbeat_delay_sec"] = self.get_parameter(
            'neighbor_management.max_heartbeat_delay_sec').value
        self.params["neighbor_management.heartbeat_period_sec"] = self.get_parameter(
            'neighbor_management.heartbeat_period_sec').value
        self.params[
            "frontend.global_descriptor_publication_period_sec"] = self.get_parameter(
                'frontend.global_descriptor_publication_period_sec').value
        self.params[
            "frontend.global_descriptor_publication_max_elems_per_msg"] = self.get_parameter(
                'frontend.global_descriptor_publication_max_elems_per_msg').value
        self.params["frontend.use_vertex_cover_selection"] = self.get_parameter(
            'frontend.use_vertex_cover_selection').value
        self.params["frontend.cosplace.descriptor_dim"] = self.get_parameter(
            'frontend.cosplace.descriptor_dim').value
        self.params["frontend.cosplace.backbone"] = self.get_parameter(
            'frontend.cosplace.backbone').value

        self.glcd = GlobalImageDescriptorLoopClosureDetection(
            self.params, self)
        self.inter_robot_detection_timer = self.create_timer(
            self.params['frontend.inter_robot_detection_period_sec'],
            self.glcd.detect_inter)


if __name__ == '__main__':

    rclpy.init(args=None)
    lcd = LoopClosureDetection()
    lcd.get_logger().info('Initialization done.')
    rclpy.spin(lcd)
    rclpy.shutdown()
