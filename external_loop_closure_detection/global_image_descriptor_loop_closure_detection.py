#!/usr/bin/env python
import numpy as np
from cv_bridge import CvBridge

import os
from os.path import join, exists, isfile, realpath, dirname
import numpy as np
import sys

from external_loop_closure_detection.nearest_neighbors_matching import NearestNeighborsMatching
from external_loop_closure_detection.netvlad import NetVLAD

from cslam_loop_detection.msg import GlobalImageDescriptor


class GlobalImageDescriptorLoopClosureDetection(object):
    def __init__(self, params, node):
        self.params = params
        self.node = node
        self.local_nnsm = NearestNeighborsMatching()
        self.counter = 0

        if self.params['technique'].lower() == 'netvlad':
            self.params['pca'] = self.node.get_parameter('pca').value
            self.global_descriptor = NetVLAD(self.params, self.node)
        else:
            self.get_logger().err('ERROR: Unknown technique. Using NetVLAD as default.')
            self.params['pca'] = self.node.get_parameter('pca').value
            self.global_descriptor = NetVLAD(self.params, self.node)

        self.params['global_descriptor_topic'] = self.node.get_parameter('global_descriptor_topic').value
        self.global_descriptor_publisher = self.node.create_publisher(
            GlobalImageDescriptor, 
            self.params['global_descriptor_topic'], 
            10)
        self.global_descriptor_subscriber = self.node.create_subscription(
            GlobalImageDescriptor, 
            self.params['global_descriptor_topic'],
            self.global_descriptor_callback,
            10)
        self.other_robots_global_descriptors = {}
        self.robot_id = self.params['robot_id']
        

    def add_keyframe(self, embedding, id):
        self.local_nnsm.add_item(embedding, id)
        msg = GlobalImageDescriptor()
        msg.image_id = id
        msg.robot_id = self.robot_id
        msg.descriptor = embedding.tolist()
        self.global_descriptor_publisher.publish(msg)

    def detect(self, embedding, id):
        kfs, ds = self.local_nnsm.search(embedding, k=self.params['nb_best_matches'])

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

    def detect_loop_closure_service(self, req, res):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(req.image.image, desired_encoding='passthrough')
        embedding = self.global_descriptor.compute_embedding(cv_image)

        # Netvlad processing
        match = None
        if self.counter > 0:
            match, best_matches = self.detect(embedding, req.image.id) # Systematic evaluation
        self.add_keyframe(embedding, req.image.id)
        self.counter = self.counter + 1

        if match is not None:
            res.is_detected=True
            res.detected_loop_closure_id=match
            res.best_matches=best_matches
        else:
            res.is_detected=False
            res.detected_loop_closure_id=-1
            res.best_matches=[]

        return res

    def global_descriptor_callback(self, msg):
        #if msg.robot_id not self.robot_id:
        if self.other_robots_global_descriptors.get(msg.robot_id):
            # Add to the NNS
            self.other_robots_global_descriptors[msg.robot_id].add_item(np.asarray(msg.descriptor), msg.image_id)
            self.node.get_logger().error('New descriptor added. ' + str(len(self.other_robots_global_descriptors[msg.robot_id].items)))
        else:
            # Create a new NNS
            self.other_robots_global_descriptors[msg.robot_id] = NearestNeighborsMatching()
            self.node.get_logger().error('New nns added.')