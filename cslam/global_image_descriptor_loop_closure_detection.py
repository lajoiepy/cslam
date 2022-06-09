#!/usr/bin/env python
from asyncio import threads
import numpy as np
from cv_bridge import CvBridge

import os
from os.path import join, exists, isfile, realpath, dirname
import numpy as np

from cslam.netvlad import NetVLAD
from cslam.loop_closure_sparse_matching import LoopClosureSparseMatching

from cslam_loop_detection.msg import GlobalImageDescriptor
from cslam_loop_detection.srv import SendLocalImageDescriptors

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
        self.counter = 0
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
            GlobalImageDescriptor, self.params['global_descriptor_topic'], 10)
        self.global_descriptor_subscriber = self.node.create_subscription(
            GlobalImageDescriptor, self.params['global_descriptor_topic'],
            self.global_descriptor_callback, 10)

        self.send_local_descriptors_srv = self.node.create_client(
            SendLocalImageDescriptors, 'send_local_image_descriptors')

    def add_keyframe(self, embedding, id):
        """ Add keyframe to matching list

        Args:
            embedding (np.array): descriptor
            id (int): keyframe ID
        """
        msg = GlobalImageDescriptor()
        msg.image_id = id
        msg.robot_id = self.robot_id
        msg.descriptor = embedding.tolist()
        # TODO: publish missing descriptors when in range
        # TODO: publish in batches of non-already transmitted
        self.global_descriptor_publisher.publish(msg)
        # Add to matches
        self.lcm.add_local_keyframe(embedding, id)

    def detect_intra(self, embedding, id):
        """ Detect intra-robot loop closures

        Args:
            embedding (np.array): descriptor
            id (int): keyframe ID

        Returns:
            list(int): matched keyframes
        """
        kfs, ds = self.local_nnsm.search(embedding,
                                         k=self.params['nb_best_matches'])

        if len(kfs) > 0 and kfs[0] == id:
            kfs, ds = kfs[1:], ds[1:]
        if len(kfs) == 0:
            return None

        for kf, d in zip(kfs, ds):
            if abs(kf - id) < self.params['min_inbetween_keyframes']:
                continue

            if d > self.params['threshold']:
                continue

            return kf, kfs
        return None, None

    def detect_inter(self):
        """ Detect inter-robot loop closures

        Returns:
            list(int): selected keyframes from other robots to match
        """
        # TODO: Find matches that maximize the algebraic connectivity
        selection = self.lcm.select_candidates(self.loop_closure_budget)
        

    def detect_loop_closure_service(self, req, res):
        """Service callback to detect loop closures associate to the keyframe 

        Args:
            req (cslam_loop_detection::srv::DetectLoopClosure::req): Keyframe data
            res (cslam_loop_detection::srv::DetectLoopClosure::res): Place recognition match data

        Returns:
            cslam_loop_detection::srv::DetectLoopClosure::res: Place recognition match data
        """
        # Netvlad processing
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(req.image.image,
                                        desired_encoding='passthrough')
        embedding = self.global_descriptor.compute_embedding(cv_image)

        # Global descriptors matching
        match = None
        if self.counter > 0: # TODO: Add param for intra-robot loop closures
            match, best_matches = self.detect_intra(
                embedding, req.image.id)  # Systematic evaluation
        self.add_keyframe(embedding, req.image.id)
        self.counter = self.counter + 1

        # Service result
        if match is not None:
            res.is_detected = True
            res.detected_loop_closure_id = match
            res.best_matches = best_matches
        else:
            res.is_detected = False
            res.detected_loop_closure_id = -1
            res.best_matches = []

        return res

    def global_descriptor_callback(self, msg):
        """Callback for descriptors received from other robots.

        Args:
            msg (cslam_loop_detection::msg::GlobalImageDescriptor): descriptor
        """
        if msg.robot_id != self.robot_id:
            self.lcm.add_other_robot_keyframe(msg)

        # TODO: Check for matches asynchronously
        #     # Match against current global descriptors
        #     match = self.detect_inter(np.asarray(msg.descriptor))

        #     # Extract and publish local descriptors
        #     if match is not None:
        #         # Call C++ code to send publish local descriptors
        #         req = SendLocalImageDescriptors.Request()
        #         req.image_id = match
        #         req.receptor_robot_id = msg.robot_id
        #         req.receptor_image_id = msg.image_id

        #         self.send_local_descriptors_srv.call_async(req)
        # TODO: if geo verif fails, remove candidate, put similarity to -1
        # TODO: if geo verif succeeds, move from candidate to fixed edge in the graph
        # TODO: Only one robot per pair should initiate computation
