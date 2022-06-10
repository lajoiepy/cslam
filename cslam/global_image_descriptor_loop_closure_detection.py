#!/usr/bin/env python
from cslam.algebraic_connectivity_maximization import EdgeInterRobot
import numpy as np
from cv_bridge import CvBridge

import os
from os.path import join, exists, isfile, realpath, dirname
import numpy as np

from cslam.netvlad import NetVLAD
from cslam.loop_closure_sparse_matching import LoopClosureSparseMatching

from cslam_common_interfaces.msg import KeyframeRGB
from cslam_loop_detection_interfaces.msg import GlobalImageDescriptor
from cslam_loop_detection_interfaces.msg import InterRobotLoopClosure
from cslam_loop_detection_interfaces.srv import SendLocalImageDescriptors

import rclpy
from rclpy.node import Node


class GlobalImageDescriptorLoopClosureDetection(object):
    """ Global Image descriptor matching """

    def __init__(self, params, node):
        """Initialization

        Args:
            params (dict): parameters
            node (ROS 2 node handle): node handle
        """
        self.params = params
        self.node = node
        self.robot_id = self.params['robot_id']

        self.lcm = LoopClosureSparseMatching(params)
        self.loop_closure_budget = self.params["loop_closure_budget"]

        # Place Recognition network setup
        if self.params['technique'].lower() == 'netvlad':
            self.params['pca'] = self.node.get_parameter('pca').value
            self.global_descriptor = NetVLAD(self.params, self.node)
        else:
            self.node.get_logger().err(
                'ERROR: Unknown technique. Using NetVLAD as default.')
            self.params['pca'] = self.node.get_parameter('pca').value
            self.global_descriptor = NetVLAD(self.params, self.node)

        # ROS 2 objects setup
        self.params['global_descriptor_topic'] = self.node.get_parameter(
            'global_descriptor_topic').value
        self.global_descriptor_publisher = self.node.create_publisher(
            GlobalImageDescriptor, self.params['global_descriptor_topic'], 100)
        self.global_descriptor_subscriber = self.node.create_subscription(
            GlobalImageDescriptor, self.params['global_descriptor_topic'],
            self.global_descriptor_callback, 100)
        self.receive_keyframe_subscriber = self.node.create_subscription(
            KeyframeRGB, 'keyframe_data', self.receive_keyframe, 100)
        self.receive_inter_robot_loop_closure_subscriber = self.node.create_subscription(
            InterRobotLoopClosure, 'inter_robot_loop_closure',
            self.receive_inter_robot_loop_closure, 100)

        self.send_local_descriptors_srv = self.node.create_client(
            SendLocalImageDescriptors, 'send_local_image_descriptors')

        self.loop_closure_list = []

    def add_keyframe(self, embedding, id):
        """ Add keyframe to matching list

        Args:
            embedding (np.array): descriptor
            id (int): keyframe ID
        """
        # Add for matching
        self.lcm.add_local_keyframe(embedding, id)

        # TODO: Maintain list to send
        msg = GlobalImageDescriptor()
        msg.image_id = id
        msg.robot_id = self.robot_id
        msg.descriptor = embedding.tolist()
        # TODO: publish missing descriptors when in range
        # TODO: publish in batches of non-already transmitted
        self.global_descriptor_publisher.publish(msg)

    def detect_intra(self, embedding, id):
        """ Detect intra-robot loop closures

        Args:
            embedding (np.array): descriptor
            id (int): keyframe ID

        Returns:
            list(int): matched keyframes
        """
        # TODO: integrate intra-robot loop closures
        kfs, ds = self.lcm.match_local_loop_closures(embedding)

    def detect_inter(self):
        """ Detect inter-robot loop closures

        Returns:
            list(int): selected keyframes from other robots to match
        """
        # TODO: specify the robots to consider for candidate selection
        # Find matches that maximize the algebraic connectivity
        selection = self.lcm.select_candidates(self.loop_closure_budget)

        # Extract and publish local descriptors
        for match in selection:
            # Call C++ code to send publish local descriptors
            req = SendLocalImageDescriptors.Request()
            req.image_id = match.robot0_image_id
            req.receptor_robot_id = match.robot1_id
            req.receptor_image_id = match.robot1_image_id

            self.send_local_descriptors_srv.call_async(req)

    def receive_keyframe(self, msg):
        """Callback to add a keyframe 

        Args:
            msg (cslam_common_interfaces::msg::KeyframeRGB): Keyframe data
        """
        # Netvlad processing
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg.image,
                                        desired_encoding='passthrough')
        embedding = self.global_descriptor.compute_embedding(cv_image)

        self.add_keyframe(embedding, msg.id)

    def global_descriptor_callback(self, msg):
        """Callback for descriptors received from other robots.

        Args:
            msg (cslam_loop_detection_interfaces::msg::GlobalImageDescriptor): descriptor
        """
        if msg.robot_id != self.robot_id:
            self.lcm.add_other_robot_keyframe(msg)

    def inter_robot_loop_closure_to_edge(self, msg):
        """Convert a inter-robot loop closure to an edge

        Args:
            msg (cslam_loop_detection_interfaces::msg::InterRobotLoopClosure): Inter-robot loop closure

        Returns:
            EdgeInterRobot: inter-robot edge
        """
        return EdgeInterRobot(msg.robot0_id, msg.robot0_image_id,
                              msg.robot1_id, msg.robot1_image_id, 
                              self.lcm.candidate_selector.fixed_weight)

    def receive_inter_robot_loop_closure(self, msg):
        """Receive computed inter-robot loop closure

        Args:
            msg (cslam_loop_detection_interfaces::msg::InterRobotLoopClosure): Inter-robot loop closure
        """
        # TODO: Only one robot per pair should initiate computation
        if msg.success:
            self.node.get_logger().info(
                'New inter-robot Loop closure measurement.')
            self.loop_closure_list.append(msg)
            # If geo verif succeeds, move from candidate to fixed edge in the graph
            self.lcm.candidate_selector.candidate_edges_to_fixed(
                list(self.inter_robot_loop_closure_to_edge(msg)))
        else:
            self.node.get_logger().info(
                'Failed inter-robot Loop closure measurement.')
            # If geo verif fails, remove candidate
            self.lcm.candidate_selector.remove_candidate_edges(
                list(self.inter_robot_loop_closure_to_edge(msg)))
