#!/usr/bin/env python
from cslam.algebraic_connectivity_maximization import EdgeInterRobot
import numpy as np

import os
from os.path import join, exists, isfile, realpath, dirname

from cslam.loop_closure_sparse_matching import LoopClosureSparseMatching
from cslam.broker import Broker

from cslam_common_interfaces.msg import KeyframeRGB, KeyframePointCloud
from cslam_common_interfaces.msg import (
    GlobalDescriptor, GlobalDescriptors, InterRobotLoopClosure,
    LocalDescriptorsRequest, LocalKeyframeMatch, InterRobotMatch,
    InterRobotMatches)
from diagnostic_msgs.msg import KeyValue
import time
from sortedcontainers import SortedDict

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from cslam.neighbors_manager import NeighborManager
from cslam.utils.misc import dict_to_list_chunks

class GlobalDescriptorLoopClosureDetection(object):
    """ Global descriptor matching """

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
        if self.params['frontend.global_descriptor_technique'].lower(
        ) == 'netvlad':
            from cslam.vpr.netvlad import NetVLAD
            self.node.get_logger().info('Using NetVLAD.')
            self.global_descriptor = NetVLAD(self.params, self.node)
            self.keyframe_type = "rgb"
        elif self.params['frontend.global_descriptor_technique'].lower(
        ) == 'scancontext':
            from cslam.lidar_pr.scancontext import ScanContext
            global icp_utils
            import cslam.lidar_pr.icp_utils as icp_utils
            self.node.get_logger().info('Using ScanContext.')
            self.global_descriptor = ScanContext(self.params, self.node)
            self.keyframe_type = "pointcloud"
        else:
            from cslam.vpr.cosplace import CosPlace
            self.node.get_logger().info('Using CosPlace. (default)')
            self.global_descriptor = CosPlace(self.params, self.node)
            self.keyframe_type = "rgb"

        # ROS 2 objects setup
        self.params[
            'frontend.global_descriptors_topic'] = '/cslam/' + self.node.get_parameter(
                'frontend.global_descriptors_topic').value
        self.global_descriptor_publisher = self.node.create_publisher(
            GlobalDescriptors,
            self.params['frontend.global_descriptors_topic'], 100)
        self.global_descriptor_subscriber = self.node.create_subscription(
            GlobalDescriptors,
            self.params['frontend.global_descriptors_topic'],
            self.global_descriptor_callback, 100)

        self.params[
            'frontend.inter_robot_matches_topic'] = '/cslam/' + self.node.get_parameter(
                'frontend.inter_robot_matches_topic').value
        self.inter_robot_matches_publisher = self.node.create_publisher(
            InterRobotMatches,
            self.params['frontend.inter_robot_matches_topic'], 100)
        self.inter_robot_matches_subscriber = self.node.create_subscription(
            InterRobotMatches,
            self.params['frontend.inter_robot_matches_topic'],
            self.inter_robot_matches_callback, 100)

        if self.keyframe_type == "rgb":
            self.receive_keyframe_subscriber = self.node.create_subscription(
                KeyframeRGB, 'cslam/keyframe_data', self.receive_keyframe, 100)
        elif self.keyframe_type == "pointcloud":
            self.receive_keyframe_subscriber = self.node.create_subscription(
                KeyframePointCloud, 'cslam/keyframe_data', self.receive_keyframe,
                100)
        else:
            self.node.get_logger().error("Unknown keyframe type")

        self.local_match_publisher = self.node.create_publisher(
            LocalKeyframeMatch, 'cslam/local_keyframe_match', 100)

        self.receive_inter_robot_loop_closure_subscriber = self.node.create_subscription(
            InterRobotLoopClosure, '/cslam/inter_robot_loop_closure',
            self.receive_inter_robot_loop_closure, 100)

        self.local_descriptors_request_publishers = {}
        for i in range(self.params['max_nb_robots']):
            self.local_descriptors_request_publishers[
                i] = self.node.create_publisher(
                    LocalDescriptorsRequest,
                    '/r' + str(i) + '/cslam/local_descriptors_request', 100)

        # Listen for changes in node liveliness
        self.neighbor_manager = NeighborManager(
            self.node, self.params)

        self.global_descriptors_buffer = SortedDict()
        self.global_descriptors_timer = self.node.create_timer(
            self.params['frontend.detection_publication_period_sec'],
            self.global_descriptors_timer_callback,
            clock=Clock()
        )  # Note: It is important to use the system clock instead of ROS clock for timers since we are within a TimerAction

        self.inter_robot_matches_buffer = SortedDict()
        self.nb_inter_robot_matches = 0
        self.inter_robot_matches_timer = self.node.create_timer(
            self.params['frontend.detection_publication_period_sec'],
            self.inter_robot_matches_timer_callback,
            clock=Clock()
        )  # Note: It is important to use the system clock instead of ROS clock for timers since we are within a TimerAction

        if self.params["evaluation.enable_logs"]:
            self.log_publisher = self.node.create_publisher(
                KeyValue, 'cslam/log_info', 100)
            self.log_matches_publisher = self.node.create_publisher(
                InterRobotMatches, 'cslam/log_matches', 100)
            self.log_total_successful_matches = 0
            self.log_total_failed_matches = 0
            self.log_total_vertices_transmitted = 0
            self.log_total_matches_selected = 0
            self.log_detection_cumulative_communication = 0
            self.log_total_sparsification_computation_time = 0.0
        
        # Import OpenCV at the end to avoid issue on NVIDIA Jetson Xavier: https://github.com/opencv/opencv/issues/14884#issuecomment-599852128
        global cv2
        import cv2
        global CvBridge
        from cv_bridge import CvBridge

        self.gpu_start_time = time.time() 

    def add_global_descriptor_to_map(self, embedding, kf_id):
        """ Add global descriptor to matching list

        Args:
            embedding (np.array): descriptor
            kf_id (int): keyframe ID
        """
        # Add for matching
        matches = self.lcm.add_local_global_descriptor(embedding, kf_id)
        # Local matching
        self.detect_intra(embedding, kf_id)

        # Store global descriptor
        msg = GlobalDescriptor()
        msg.keyframe_id = kf_id
        msg.robot_id = self.params['robot_id']
        msg.descriptor = embedding.tolist()
        self.global_descriptors_buffer[kf_id] = msg

        # Store matches
        for match in matches:
            self.inter_robot_matches_buffer[
                self.nb_inter_robot_matches] = match
            self.nb_inter_robot_matches += 1

    def delete_useless_descriptors(self):
        """Deletes global descriptors
           because all other robots have already received them.
        """
        from_kf_id = self.neighbor_manager.useless_descriptors(
            self.global_descriptors_buffer.peekitem(-1)[0])
        if from_kf_id >= self.global_descriptors_buffer.peekitem(0)[0]:
            for k in self.global_descriptors_buffer.keys():
                if k < from_kf_id:
                    del self.global_descriptors_buffer[k]

    def delete_useless_inter_robot_matches(self):
        """Deletes inter_robot_matches
           because all other robots have already received them.
        """
        from_match_id = self.neighbor_manager.useless_matches(
            self.inter_robot_matches_buffer.peekitem(-1)[0])
        if from_match_id >= self.inter_robot_matches_buffer.peekitem(0)[0]:
            for k in self.inter_robot_matches_buffer.keys():
                if k < from_match_id:
                    del self.inter_robot_matches_buffer[k]

    def global_descriptors_timer_callback(self):
        """Publish global descriptors message periodically
        Doesn't publish if the descriptors are already known by neighboring robots
        """
        if len(self.global_descriptors_buffer) > 0:
            from_kf_id = self.neighbor_manager.select_from_which_kf_to_send(
                self.global_descriptors_buffer.peekitem(-1)[0])

            msgs = dict_to_list_chunks(
                self.global_descriptors_buffer,
                from_kf_id - self.global_descriptors_buffer.peekitem(0)[0],
                self.params['frontend.detection_publication_max_elems_per_msg']
            )

            for m in msgs:
                global_descriptors = GlobalDescriptors()
                global_descriptors.descriptors = m
                self.global_descriptor_publisher.publish(global_descriptors)
                if self.params["evaluation.enable_logs"]:
                    self.log_detection_cumulative_communication += len(
                        global_descriptors.descriptors) * len(
                            global_descriptors.descriptors[0].descriptor
                        ) * 4  # bytes

            self.delete_useless_descriptors()
            if self.params["evaluation.enable_logs"]:
                self.log_publisher.publish(
                    KeyValue(key="detection_cumulative_communication",
                             value=str(
                                 self.log_detection_cumulative_communication)))

    def edge_to_match(self, edge):
        """Converts an InterRobotEdge to a InterRobotMatch message
           Args: edge (InterRobotEdge)
        """
        msg = InterRobotMatch()
        msg.robot0_id = edge.robot0_id
        msg.robot0_keyframe_id = edge.robot0_keyframe_id
        msg.robot1_id = edge.robot1_id
        msg.robot1_keyframe_id = edge.robot1_keyframe_id
        msg.weight = edge.weight
        return msg

    def inter_robot_matches_timer_callback(self):
        """Publish inter-robot matches message periodically
        Doesn't publish if the inter-robot matches are already known by neighboring robots
        """
        if len(self.inter_robot_matches_buffer) > 0:
            from_match_idx = self.neighbor_manager.select_from_which_match_to_send(
                self.inter_robot_matches_buffer.peekitem(-1)[0])

            chuncks = dict_to_list_chunks(
                self.inter_robot_matches_buffer, from_match_idx -
                self.inter_robot_matches_buffer.peekitem(0)[0], self.
                params['frontend.detection_publication_max_elems_per_msg'])

            # Don't transmit matches that should have already been detected by the other robot
            _, neighbors_in_range_list = self.neighbor_manager.check_neighbors_in_range()
            if len(neighbors_in_range_list) == 2:
                self.node.get_logger().info("Transmitting matches {}".format(neighbors_in_range_list))
                for c in chuncks:
                    for match in c:
                        if match.robot0_id in neighbors_in_range_list and match.robot1_id in neighbors_in_range_list:
                            c.remove(match)
                    if len(c) <= 0:
                        chuncks.remove(c)

            # Convert to ROS message
            msgs = []
            for c in chuncks:
                m = []
                for match in c:
                    msg = self.edge_to_match(match)
                    m.append(msg)
                msgs.append(m)

            # Transmit the rest
            for m in msgs:
                inter_robot_matches = InterRobotMatches()
                inter_robot_matches.robot_id = self.params['robot_id']
                inter_robot_matches.matches = m
                self.inter_robot_matches_publisher.publish(inter_robot_matches)
                if self.params["evaluation.enable_logs"]:
                    self.log_detection_cumulative_communication += len(
                        inter_robot_matches.matches) * 20  # bytes

            self.delete_useless_inter_robot_matches()
            if self.params["evaluation.enable_logs"]:
                self.log_publisher.publish(
                    KeyValue(key="detection_cumulative_communication",
                             value=str(
                                 self.log_detection_cumulative_communication)))

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
        #self.node.get_logger().info('Neighbors in range: ' +  str(neighbors_in_range_list))
        # Check if the robot is the broker
        if len(neighbors_in_range_list
               ) > 0 and self.neighbor_manager.local_robot_is_broker():
            if self.params["evaluation.enable_logs"]: start_time = time.time()
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
                    msg.keyframe_id = v[1]
                    msg.matches_robot_id = vertices_info[v][0]
                    msg.matches_keyframe_id = vertices_info[v][1]
                    self.local_descriptors_request_publishers[v[0]].publish(
                        msg)
                if self.params["evaluation.enable_logs"]:
                    self.log_total_vertices_transmitted += len(
                        selected_vertices_set)
            if self.params["evaluation.enable_logs"]:
                stop_time = time.time()
                self.log_total_sparsification_computation_time += stop_time - start_time
                self.log_total_matches_selected += len(selection)
                self.log_publisher.publish(
                    KeyValue(
                        key="sparsification_cumulative_computation_time",
                        value=str(
                            self.log_total_sparsification_computation_time)))
                self.log_publisher.publish(
                    KeyValue(key="nb_vertices_transmitted",
                             value=str(self.log_total_vertices_transmitted)))
                self.log_publisher.publish(
                    KeyValue(key="nb_matches_selected",
                             value=str(self.log_total_matches_selected)))
                if self.params["evaluation.enable_sparsification_comparison"]:
                    matches = InterRobotMatches()
                    matches.robot_id = self.params["robot_id"]
                    for e in self.lcm.candidate_selector.log_mac_edges:
                        matches.matches.append(self.edge_to_match(e))
                    self.log_matches_publisher.publish(matches)

    def edge_list_to_vertices(self, selection):
        """Extracts the vertices in a list of edges
        Args:
            selection list(EdgeInterRobot): selection of edges
        Returns:
            dict((int, int), list(int), list(int)): Vertices indices with their related vertices
        """
        vertices = {}
        for s in selection:
            key0 = (s.robot0_id, s.robot0_keyframe_id)
            key1 = (s.robot1_id, s.robot1_keyframe_id)
            if key0 in vertices:
                vertices[key0][0].append(s.robot1_id)
                vertices[key0][1].append(s.robot1_keyframe_id)
            else:
                vertices[key0] = [[s.robot1_id], [s.robot1_keyframe_id]]
            if key1 in vertices:
                vertices[key1][0].append(s.robot0_id)
                vertices[key1][1].append(s.robot0_keyframe_id)
            else:
                vertices[key1] = [[s.robot0_id], [s.robot0_keyframe_id]]
        return vertices

    def receive_keyframe(self, msg):
        """Callback to add a keyframe 

        Args:
            msg (cslam_common_interfaces::msg::KeyframeRGB or KeyframePointCloud): Keyframe data
        """
        # Place recognition descriptor processing
        embedding = []
        if self.keyframe_type == "rgb":
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg.image,
                                            desired_encoding='passthrough')
            embedding = self.global_descriptor.compute_embedding(cv_image)
        elif self.keyframe_type == "pointcloud":
            embedding = self.global_descriptor.compute_embedding(
                icp_utils.ros_pointcloud_to_points(msg.pointcloud))

        self.add_global_descriptor_to_map(embedding, msg.id)

    def global_descriptor_callback(self, msg):
        """Callback for descriptors received from other robots.

        Args:
            msg (cslam_common_interfaces::msg::GlobalDescriptors): descriptors
        """
        if msg.descriptors[0].robot_id != self.params['robot_id']:
            unknown_range = self.neighbor_manager.get_unknown_range(
                msg.descriptors)
            for i in unknown_range:
                match = self.lcm.add_other_robot_global_descriptor(
                    msg.descriptors[i])
                if match is not None:
                    self.inter_robot_matches_buffer[
                        self.nb_inter_robot_matches] = match
                    self.nb_inter_robot_matches += 1

    def inter_robot_matches_callback(self, msg):
        """Callback for inter-robot matches received from other robots.

        Args:
            msg (cslam_common_interfaces::msg::InterRobotMatches): matches
        """
        if msg.robot_id != self.params['robot_id']:
            for match in msg.matches:
                edge = EdgeInterRobot(match.robot0_id, match.robot0_keyframe_id, match.robot1_id, match.robot1_keyframe_id, match.weight)
                self.lcm.candidate_selector.add_match(edge)

    def inter_robot_loop_closure_msg_to_edge(self, msg):
        """ Convert a inter-robot loop closure to an edge 
            for algebraic connectivity maximization

        Args:
            msg (cslam_common_interfaces::msg::InterRobotLoopClosure): Inter-robot loop closure

        Returns:
            EdgeInterRobot: inter-robot edge
        """
        return EdgeInterRobot(msg.robot0_id, msg.robot0_keyframe_id,
                              msg.robot1_id, msg.robot1_keyframe_id,
                              self.lcm.candidate_selector.fixed_weight)

    def receive_inter_robot_loop_closure(self, msg):
        """Receive computed inter-robot loop closure

        Args:
            msg (cslam_common_interfaces::msg::InterRobotLoopClosure): Inter-robot loop closure
        """
        if msg.success:
            self.node.get_logger().info(
                'New inter-robot loop closure measurement: (' +
                str(msg.robot0_id) + ',' + str(msg.robot0_keyframe_id) +
                ') -> (' + str(msg.robot1_id) + ',' +
                str(msg.robot1_keyframe_id) + ')')
            # If geo verif succeeds, move from candidate to fixed edge in the graph
            self.lcm.candidate_selector.candidate_edges_to_fixed(
                [self.inter_robot_loop_closure_msg_to_edge(msg)])

            if self.params["evaluation.enable_logs"]:
                self.log_total_successful_matches += 1
                self.log_publisher.publish(
                    KeyValue(key="nb_matches",
                             value=str(self.log_total_successful_matches)))
        else:
            # If geo verif fails, remove candidate
            self.node.get_logger().info(
                'Failed inter-robot loop closure measurement: (' +
                str(msg.robot0_id) + ',' + str(msg.robot0_keyframe_id) +
                ') -> (' + str(msg.robot1_id) + ',' +
                str(msg.robot1_keyframe_id) + ')')
            self.lcm.candidate_selector.remove_candidate_edges(
                [self.inter_robot_loop_closure_msg_to_edge(msg)], failed=True)

            if self.params["evaluation.enable_logs"]:
                self.log_total_failed_matches += 1
                self.log_publisher.publish(
                    KeyValue(key="nb_failed_matches",
                             value=str(self.log_total_failed_matches)))
