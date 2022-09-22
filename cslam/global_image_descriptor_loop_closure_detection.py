#!/usr/bin/env python
from cslam.algebraic_connectivity_maximization import EdgeInterRobot
import numpy as np
from cv_bridge import CvBridge

import os
from os.path import join, exists, isfile, realpath, dirname
import numpy as np

from cslam.vpr.netvlad import NetVLAD
from cslam.vpr.cosplace import CosPlace
from cslam.loop_closure_sparse_matching import LoopClosureSparseMatching
from cslam.broker import Broker

from cslam_common_interfaces.msg import KeyframeRGB
from cslam_loop_detection_interfaces.msg import (GlobalImageDescriptor,
                                                 GlobalImageDescriptors,
                                                 InterRobotLoopClosure,
                                                 LocalDescriptorsRequest,
                                                 LocalKeyframeMatch)

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from cslam.neighbors_manager import NeighborManager
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
        self.lcm = LoopClosureSparseMatching(params)

        # Place Recognition network setup
        pkg_folder = get_package_share_directory("cslam")
        self.params['frontend.nn_checkpoint'] = join(pkg_folder, self.params['frontend.nn_checkpoint'])
        if self.params['frontend.global_descriptor_technique'].lower() == 'cosplace':
            self.node.get_logger().info(
                'Using CosPlace.')
            self.global_descriptor = CosPlace(self.params, self.node)            
        else:
            self.node.get_logger().info(
                'Using NetVLAD (default).')
            self.params['frontend.netvlad.pca_checkpoint'] = join(pkg_folder, self.node.get_parameter(
                'frontend.netvlad.pca_checkpoint').value)
            self.global_descriptor = NetVLAD(self.params, self.node)

        # ROS 2 objects setup
        self.params[
            'frontend.global_descriptor_topic'] = self.node.get_parameter(
                'frontend.global_descriptor_topic').value
        self.global_descriptor_publisher = self.node.create_publisher(
            GlobalImageDescriptors,
            self.params['frontend.global_descriptor_topic'], 100)
        self.global_descriptor_subscriber = self.node.create_subscription(
            GlobalImageDescriptors,
            self.params['frontend.global_descriptor_topic'],
            self.global_descriptor_callback, 100)
        self.receive_keyframe_subscriber = self.node.create_subscription(
            KeyframeRGB, 'keyframe_data', self.receive_keyframe, 100)

        self.local_match_publisher = self.node.create_publisher(
            LocalKeyframeMatch, 'local_keyframe_match', 100)

        self.receive_inter_robot_loop_closure_subscriber = self.node.create_subscription(
            InterRobotLoopClosure, '/inter_robot_loop_closure',
            self.receive_inter_robot_loop_closure, 100)

        self.local_descriptors_request_publishers = {}
        for i in range(self.params['nb_robots']):
            self.local_descriptors_request_publishers[
                i] = self.node.create_publisher(
                    LocalDescriptorsRequest,
                    '/r' + str(i) + '/local_descriptors_request', 100)

        # Listen for changes in node liveliness
        self.neighbor_manager = NeighborManager(
            self.node, self.params['robot_id'], self.params['nb_robots'],
            self.params['neighbor_management.enable_neighbor_monitoring'],
            self.params['neighbor_management.max_heartbeat_delay_sec'])

        self.global_descriptors_buffer = []
        self.global_descriptors_timer = self.node.create_timer(
            self.params['frontend.global_descriptor_publication_period_sec'],
            self.global_descriptors_timer_callback)

    def add_global_descriptor_to_map(self, embedding, kf_id):
        """ Add global descriptor to matching list

        Args:
            embedding (np.array): descriptor
            kf_id (int): keyframe ID
        """
        # Add for matching
        self.lcm.add_local_global_descriptor(embedding, kf_id)
        # Local matching
        self.detect_intra(embedding, kf_id)

        # Store global descriptor
        msg = GlobalImageDescriptor()
        msg.image_id = kf_id
        msg.robot_id = self.params['robot_id']
        msg.descriptor = embedding.tolist()
        self.global_descriptors_buffer.append(msg)

    def delete_useless_descriptors(self):
        """Deletes global descriptors
           because all other robots have already received 
           some descriptors
        """
        from_kf_id = self.neighbor_manager.useless_descriptors(
            self.global_descriptors_buffer[-1].image_id)
        if from_kf_id >= self.global_descriptors_buffer[0].image_id:
            self.global_descriptors_buffer = [
                e for e in self.global_descriptors_buffer
                if e.image_id > from_kf_id
            ]

    def global_descriptors_timer_callback(self):
        """Publish global descriptors message periodically
        """
        if len(self.global_descriptors_buffer) > 0:
            from_kf_id = self.neighbor_manager.select_from_which_kf_to_send(
                self.global_descriptors_buffer[-1].image_id)

            msgs = list_chunks(
                self.global_descriptors_buffer,
                from_kf_id - self.global_descriptors_buffer[0].image_id,
                self.params[
                    'frontend.global_descriptor_publication_max_elems_per_msg']
            )

            for m in msgs:
                global_descriptors = GlobalImageDescriptors()
                global_descriptors.descriptors = m
                self.global_descriptor_publisher.publish(global_descriptors)

            self.delete_useless_descriptors()

    def detect_intra(self, embedding, kf_id):
        """ Detect intra-robot loop closures

        Args:
            embedding (np.array): descriptor
            kf_id (int): keyframe ID

        Returns:
            list(int): matched keyframes
        """
        if self.params['frontend.enable_intra_robot_loop_closures']:
            kf_match, _ = self.lcm.match_local_loop_closures(embedding, kf_id)
            if kf_match is not None:
                msg = LocalKeyframeMatch()
                msg.keyframe0_id = kf_id
                msg.keyframe1_id = kf_match
                self.local_match_publisher.publish(msg)

    def detect_inter(self):
        """ Detect inter-robot loop closures

        Returns:
            list(int): selected keyframes from other robots to match
        """
        neighbors_is_in_range, neighbors_in_range_list = self.neighbor_manager.check_neighbors_in_range(
        )
        # Check if the robot is the broker
        if len(neighbors_in_range_list
               ) > 0 and self.neighbor_manager.local_robot_is_broker():
            # Find matches that maximize the algebraic connectivity
            selection = self.lcm.select_candidates(
                self.params["frontend.inter_robot_loop_closure_budget"],
                neighbors_is_in_range)          

            # Extract and publish local descriptors
            vertices_info = self.edge_list_to_vertices(selection)
            broker = Broker(selection, neighbors_in_range_list)
            for selected_vertices_set in broker.brokerage(
                    self.params["frontend.use_vertex_cover_selection"]):
                for v in selected_vertices_set:
                    # Call to send publish local descriptors
                    msg = LocalDescriptorsRequest()
                    msg.image_id = v[1]
                    msg.matches_robot_id = vertices_info[v][0]
                    msg.matches_image_id = vertices_info[v][1]
                    self.local_descriptors_request_publishers[v[0]].publish(
                        msg)

    def edge_list_to_vertices(self, selection):
        """Extracts the vertices in a list of edges
        Args:
            selection list(EdgeInterRobot): selection of edges
        Returns:
            dict((int, int), list(int), list(int)): Vertices indices with their related vertices
        """
        vertices = {}
        for s in selection:
            key0 = (s.robot0_id, s.robot0_image_id)
            key1 = (s.robot1_id, s.robot1_image_id)
            if key0 in vertices:
                vertices[key0][0].append(s.robot1_id)
                vertices[key0][1].append(s.robot1_image_id)
            else:
                vertices[key0] = [[s.robot1_id], [s.robot1_image_id]]
            if key1 in vertices:
                vertices[key1][0].append(s.robot0_id)
                vertices[key1][1].append(s.robot0_image_id)
            else:
                vertices[key1] = [[s.robot0_id], [s.robot0_image_id]]
        return vertices

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

        self.add_global_descriptor_to_map(embedding, msg.id)

    def global_descriptor_callback(self, msg):
        """Callback for descriptors received from other robots.

        Args:
            msg (cslam_loop_detection_interfaces::msg::GlobalImageDescriptors): descriptors
        """
        if msg.descriptors[0].robot_id != self.params['robot_id']:
            unknown_range = self.neighbor_manager.get_unknown_range(
                msg.descriptors)
            for i in unknown_range:
                self.lcm.add_other_robot_global_descriptor(msg.descriptors[i])

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
        if msg.success:
            self.node.get_logger().info(
                'New inter-robot loop closure measurement: (' +
                str(msg.robot0_id) + ',' + str(msg.robot0_image_id) +
                ') -> (' + str(msg.robot1_id) + ',' +
                str(msg.robot1_image_id) + ')')
            # If geo verif succeeds, move from candidate to fixed edge in the graph
            self.lcm.candidate_selector.candidate_edges_to_fixed(
                [self.inter_robot_loop_closure_msg_to_edge(msg)])
        else:
            self.node.get_logger().info(
                'Failed inter-robot loop closure measurement: (' +
                str(msg.robot0_id) + ',' + str(msg.robot0_image_id) +
                ') -> (' + str(msg.robot1_id) + ',' +
                str(msg.robot1_image_id) + ')')
            # If geo verif fails, remove candidate
            self.lcm.candidate_selector.remove_candidate_edges(
                [self.inter_robot_loop_closure_msg_to_edge(msg)])