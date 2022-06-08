#!/usr/bin/env python
from asyncio import threads
import numpy as np
from cv_bridge import CvBridge

import os
from os.path import join, exists, isfile, realpath, dirname
import numpy as np
from scipy.stats import logistic

from cslam.nearest_neighbors_matching import NearestNeighborsMatching
from cslam.netvlad import NetVLAD
from cslam.algebraic_connectivity_maximization import AlgebraicConnectivityMaximization

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

        # Multi-robot setup
        self.robot_id = self.params['robot_id']
        self.local_nnsm = NearestNeighborsMatching()
        self.other_robots_nnsm = {}
        self.best_matches = {'local_keyframe_id': []}
        self.nb_robots = self.params['nb_robots']
        for i in range(self.nb_robots):
            if i != self.robot_id:
                self.other_robots_nnsm[i] = NearestNeighborsMatching()
                self.best_matches['robot_' + str(i) + '_image_id'] = []
                self.best_matches['robot_' + str(i) + '_similarity'] = []
        self.similarity_loc = self.params['similarity_loc']
        self.similarity_scale = self.params['similarity_scale']
        self.loop_closure_budget = self.params["loop_closure_budget"]

        self.counter = 0

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

    def distance_to_similarity(self, distance):
        """Converts a distance metric into a similarity score

        Args:
            distance (float): Place recognition distance metric

        Returns:
            float: similarity score
        """
        return logistic.cdf(-distance,
                            loc=self.similarity_loc,
                            scale=self.similarity_scale)

    def add_keyframe(self, embedding, id):
        """ Add keyframe to matching list

        Args:
            embedding (np.array): descriptor
            id (int): keyframe ID
        """
        self.local_nnsm.add_item(embedding, id)
        msg = GlobalImageDescriptor()
        msg.image_id = id
        msg.robot_id = self.robot_id
        msg.descriptor = embedding.tolist()
        # TODO: publish missing descriptors when in range
        # TODO: publish in batches of non-already transmitted
        self.global_descriptor_publisher.publish(msg)
        # Add to best matches list
        self.best_matches['local_keyframe_id'].append(id)
        for i in range(self.nb_robots):
            if i != self.robot_id:
                kf, d = self.other_robots_nnsm[i].search_best(embedding, k=1)
                if d <= self.params['threshold']:
                    self.best_matches['robot_' + str(i) +
                                      '_image_id'].append(kf)
                    self.best_matches['robot_' + str(i) +
                                      '_similarity'].append(
                                          self.distance_to_similarity(d))
                else:
                    self.best_matches['robot_' + str(i) +
                                      '_image_id'].append(-1)
                    self.best_matches['robot_' + str(i) +
                                      '_similarity'].append(-1)

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

        Args:
            embedding (np.array): descriptor

        Returns:
            list(int): matched keyframes from other robots
        """
        # TODO: Find matches that maximize the algebraic connectivity
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list, nb_poses)
        selection = ac.select_candidates_random_initialization(
            nb_candidates_to_choose)

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
        if self.counter > 0:
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
            self.other_robots_nnsm[msg.robot_id].add_item(
                np.asarray(msg.descriptor), msg.image_id)

            kf, d = self.local_nnsm.search_best(np.asarray(msg.descriptor))
            similarity = self.distance_to_similarity(d)
            if d <= self.params['threshold'] and similarity > self.best_matches[
                    'robot_' + str(msg.robot_id) + '_similarity'][kf]:
                self.best_matches['robot_' + str(msg.robot_id) +
                                  '_image_id'][kf] = msg.image_id
                self.best_matches['robot_' + str(msg.robot_id) +
                                  '_similarity'][kf] = similarity

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
