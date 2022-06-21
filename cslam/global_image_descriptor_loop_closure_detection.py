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
from cslam_loop_detection_interfaces.msg import GlobalImageDescriptor, GlobalImageDescriptors
from cslam_loop_detection_interfaces.msg import InterRobotLoopClosure
from cslam_loop_detection_interfaces.srv import SendLocalImageDescriptors

import rclpy
from rclpy.node import Node

from cslam.neighbors_manager import NeighborManager
from test_msgs.msg import Empty as EmptyMsg
from cslam.utils.utils import list_chunks

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
        self.nb_robots = self.params['nb_robots']

        self.lcm = LoopClosureSparseMatching(params)
        self.loop_closure_budget = self.params["loop_closure_budget"]

        # Place Recognition network setup
        if self.params['global_descriptor_technique'].lower() == 'netvlad':
            self.params['pca_checkpoint'] = self.node.get_parameter(
                'pca_checkpoint').value
            self.global_descriptor = NetVLAD(self.params, self.node)
        else:
            self.node.get_logger().err(
                'ERROR: Unknown technique. Using NetVLAD as default.')
            self.params['pca_checkpoint'] = self.node.get_parameter(
                'pca_checkpoint').value
            self.global_descriptor = NetVLAD(self.params, self.node)

        # ROS 2 objects setup
        self.params['global_descriptor_topic'] = self.node.get_parameter(
            'global_descriptor_topic').value
        self.global_descriptor_publisher = self.node.create_publisher(
            GlobalImageDescriptors, self.params['global_descriptor_topic'], 100)
        self.global_descriptor_subscriber = self.node.create_subscription(
            GlobalImageDescriptors, self.params['global_descriptor_topic'],
            self.global_descriptor_callback, 100)
        self.receive_keyframe_subscriber = self.node.create_subscription(
            KeyframeRGB, 'keyframe_data', self.receive_keyframe, 100)
        self.receive_inter_robot_loop_closure_subscriber = self.node.create_subscription(
            InterRobotLoopClosure, 'inter_robot_loop_closure',
            self.receive_inter_robot_loop_closure, 100)

        self.send_local_descriptors_srv = self.node.create_client(
            SendLocalImageDescriptors, 'send_local_image_descriptors')

        self.loop_closure_list = []

        # Listen for changes in node liveliness
        self.alive_publisher = self.node.create_publisher(EmptyMsg, 'alive', 10)
        self.neighbor_manager = NeighborManager(self.node, self.robot_id, self.nb_robots, self.params['max_alive_delay_sec'])

        self.alive_timer = self.node.create_timer(self.params['alive_check_period_sec'], self.alive_timer_callback)

        self.global_descriptors_buffer = []
        self.global_descriptors_timer = self.node.create_timer(self.params['global_descriptor_publication_period'], self.global_descriptors_timer_callback)


    def alive_timer_callback(self):
        """Publish alive messagee periodically
        """
        self.alive_publisher.publish(EmptyMsg())

    def add_keyframe(self, embedding, id):
        """ Add keyframe to matching list

        Args:
            embedding (np.array): descriptor
            id (int): keyframe ID
        """
        # Add for matching
        self.lcm.add_local_keyframe(embedding, id)

        # Store global descriptor
        msg = GlobalImageDescriptor()
        msg.image_id = id
        msg.robot_id = self.robot_id
        msg.descriptor = embedding.tolist()
        self.global_descriptors_buffer.append(msg)

    def delete_useless_descriptors(self):
        """Deletes global descriptors
           because all other robots have already received 
           some descriptors
        """
        from_kf_id = self.neighbor_manager.useless_descriptors(self.global_descriptors_buffer[-1].image_id)
        if from_kf_id > self.global_descriptors_buffer[0].image_id:
            self.global_descriptors_buffer = [e for e in self.global_descriptors_buffer if e.image_id > from_kf_id]

    def global_descriptors_timer_callback(self):
        """Publish global descriptors message periodically
        """
        if len(self.global_descriptors_buffer) > 0:
            from_kf_id = self.neighbor_manager.select_from_which_kf_to_send(self.global_descriptors_buffer[-1].image_id)

            msgs = list_chunks(self.global_descriptors_buffer, from_kf_id, self.params['global_descriptor_publication_max_elems_per_msg'])

            for m in msgs:
                self.global_descriptor_publisher.publish(m)
            
            self.delete_useless_descriptors()

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
        # Find matches that maximize the algebraic connectivity
        selection = self.lcm.select_candidates(self.loop_closure_budget, self.neighbor_manager.check_neighbors_in_range())

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
            msg (cslam_loop_detection_interfaces::msg::GlobalImageDescriptors): descriptor
        """
        unknown_range = self.neighbor_manager.get_unknown_range(msg[0].image_id, msg[-1].image_id, msg[0].robot_id)
        for i in unknown_range:
            if msg[i].robot_id != self.robot_id:
                self.lcm.add_other_robot_keyframe(msg[i])

    def inter_robot_loop_closure_msg_to_edge(self, msg):
        """ Convert a inter-robot loop closure to an edge 
            for algebraic connectivity maximization

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
                'New inter-robot loop closure measurement.')
            self.loop_closure_list.append(msg)
            # If geo verif succeeds, move from candidate to fixed edge in the graph
            self.lcm.candidate_selector.candidate_edges_to_fixed(
                [self.inter_robot_loop_closure_msg_to_edge(msg)])
        else:
            self.node.get_logger().info(
                'Failed inter-robot loop closure measurement.')
            # If geo verif fails, remove candidate
            self.lcm.candidate_selector.remove_candidate_edges(
                [self.inter_robot_loop_closure_msg_to_edge(msg)])
