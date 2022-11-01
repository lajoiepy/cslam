import unittest

from cslam.broker import Broker
from cslam.algebraic_connectivity_maximization import AlgebraicConnectivityMaximization, EdgeInterRobot
from cslam.loop_closure_sparse_matching import LoopClosureSparseMatching
from test_algebraic_connectivity import build_multi_robot_graph
import random
import numpy as np
import networkx as nx
import scipy
from collections import namedtuple
import math


def build_graph_and_extract_selection(nb_poses, nb_candidate_edges, max_nb_robots,
                                      robot_id, nb_candidates_to_choose):
    """Build a graph and perform selection based on algebraic connectivity
    """
    fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
        nb_poses, nb_candidate_edges, max_nb_robots)

    params = {}
    params['robot_id'] = robot_id
    params['max_nb_robots'] = max_nb_robots
    params['frontend.similarity_threshold'] = 0.0  # Doesn't change anything
    params['frontend.sensor_type'] = 'stereo'
    params["frontend.enable_sparsification"] = True
    params["evaluation.enable_sparsification_comparison"] = False
    lcsm = LoopClosureSparseMatching(params)
    lcsm.candidate_selector.set_graph(fixed_edges_list, candidate_edges_list)

    # Solve the initial graph
    is_robot_considered = {}
    for i in range(max_nb_robots):
        is_robot_considered[i] = True
    return lcsm.select_candidates(nb_candidates_to_choose,
                                  is_robot_considered,
                                  greedy_initialization=False)


def verif_broker(unittest_framework, nb_poses, nb_candidate_edges, max_nb_robots,
                 robot_id, nb_candidates_to_choose, use_vertex_cover):
    """Test broker based on params
    """
    # Build graph
    selection = build_graph_and_extract_selection(nb_poses, nb_candidate_edges,
                                                  max_nb_robots, robot_id,
                                                  nb_candidates_to_choose)
    unittest_framework.assertEqual(
        len(selection), min(nb_candidate_edges, nb_candidates_to_choose))
    neighbors_in_range_list = range(max_nb_robots)

    # Brokerage
    broker = Broker(selection, neighbors_in_range_list)
    components = broker.brokerage(use_vertex_cover)

    # Extract vertices
    initial_vertices = set()
    duplicates = []
    for e in selection:
        v0 = (e.robot0_id, e.robot0_keyframe_id)
        if v0 in initial_vertices:
            duplicates.append(v0)
        initial_vertices.add(v0)
        v1 = (e.robot1_id, e.robot1_keyframe_id)
        if v1 in initial_vertices:
            duplicates.append(v1)
        initial_vertices.add(v1)
    # Sanity check
    unittest_framework.assertEqual(
        len(initial_vertices) + len(duplicates),
        min(nb_candidate_edges, nb_candidates_to_choose) * 2)

    vertices = []
    for c in components:
        for v in c:
            vertices.append(v)

    # Check that we do not send more vertices than the trivial solution
    # Trivial solution: sending one vertex per edge
    # Upper Bounds:
    unittest_framework.assertLessEqual(
        len(vertices), min(nb_candidate_edges, nb_candidates_to_choose))
    if use_vertex_cover and max_nb_robots == 2:
        unittest_framework.assertLessEqual(
            len(vertices), math.ceil(float(len(initial_vertices)) / 2))
    # Lower Bounds:
    unittest_framework.assertGreaterEqual(len(vertices), 1)

    # Check no duplicates
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                unittest_framework.assertNotEqual(vertices[i], vertices[j])

    # Check that all edges are covered
    for e in selection:
        v0 = (e.robot0_id, e.robot0_keyframe_id)
        v1 = (e.robot1_id, e.robot1_keyframe_id)
        v0_in = v0 in vertices
        v1_in = v1 in vertices
        unittest_framework.assertTrue(v0_in or v1_in)


class TestBroker(unittest.TestCase):
    """Unit tests for communication broker
    """

    def test_simple_dialog_2robots(self):
        """Simple dialog strategy
        """
        # Parameters
        robot_id = 0
        nb_poses = 100
        nb_candidate_edges = 50
        max_nb_robots = 2
        nb_candidates_to_choose = 30
        use_vertex_cover = False

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     nb_candidates_to_choose, use_vertex_cover)

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     nb_candidate_edges, use_vertex_cover)

        verif_broker(self, nb_poses * 10, nb_candidate_edges * 10, max_nb_robots,
                     robot_id, nb_candidates_to_choose * 10, use_vertex_cover)

        verif_broker(self, nb_poses * 10, nb_candidate_edges * 10, max_nb_robots,
                     robot_id, nb_candidate_edges * 10, use_vertex_cover)

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     2 * nb_candidate_edges, use_vertex_cover)

    def test_vertex_cover_2robots(self):
        """Vertex cover strategy
        """
        # Parameters
        robot_id = 0
        nb_poses = 100
        nb_candidate_edges = 50
        max_nb_robots = 2
        nb_candidates_to_choose = 30
        use_vertex_cover = True

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     nb_candidates_to_choose, use_vertex_cover)

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     nb_candidate_edges, use_vertex_cover)

        verif_broker(self, nb_poses * 10, nb_candidate_edges * 10, max_nb_robots,
                     robot_id, nb_candidates_to_choose * 10, use_vertex_cover)

        verif_broker(self, nb_poses * 10, nb_candidate_edges * 10, max_nb_robots,
                     robot_id, nb_candidate_edges * 10, use_vertex_cover)

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     2 * nb_candidate_edges, use_vertex_cover)

    def test_simple_dialog_5robots(self):
        """Simple dialog strategy
        """
        # Parameters
        robot_id = 1
        nb_poses = 100
        nb_candidate_edges = 200
        max_nb_robots = 5
        nb_candidates_to_choose = 100
        use_vertex_cover = False

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     nb_candidates_to_choose, use_vertex_cover)

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     nb_candidate_edges, use_vertex_cover)

        verif_broker(self, nb_poses * 10, nb_candidate_edges * 10, max_nb_robots,
                     robot_id, nb_candidates_to_choose * 10, use_vertex_cover)

        verif_broker(self, nb_poses * 10, nb_candidate_edges * 10, max_nb_robots,
                     robot_id, nb_candidate_edges * 10, use_vertex_cover)

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     2 * nb_candidate_edges, use_vertex_cover)

    def test_vertex_cover_5robots(self):
        """Vertex cover strategy
        """
        # Parameters
        robot_id = 2
        nb_poses = 100
        nb_candidate_edges = 200
        max_nb_robots = 5
        nb_candidates_to_choose = 100
        use_vertex_cover = True

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     nb_candidates_to_choose, use_vertex_cover)

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     nb_candidate_edges, use_vertex_cover)

        verif_broker(self, nb_poses * 10, nb_candidate_edges * 10, max_nb_robots,
                     robot_id, nb_candidates_to_choose * 10, use_vertex_cover)

        verif_broker(self, nb_poses * 10, nb_candidate_edges * 10, max_nb_robots,
                     robot_id, nb_candidate_edges * 10, use_vertex_cover)

        verif_broker(self, nb_poses, nb_candidate_edges, max_nb_robots, robot_id,
                     2 * nb_candidate_edges, use_vertex_cover)

    def test_manual_vertex_cover(self):
        # Build graph
        fixed_edges_list = [] 
        candidate_edges_list = [] 
        fixed_weight = 1.0
        candidate_edges_list.append(EdgeInterRobot(0, 1, 1, 1, fixed_weight))
        candidate_edges_list.append(EdgeInterRobot(0, 1, 1, 2, fixed_weight))
        candidate_edges_list.append(EdgeInterRobot(0, 1, 1, 3, fixed_weight))
        candidate_edges_list.append(EdgeInterRobot(0, 1, 1, 4, fixed_weight))
        candidate_edges_list.append(EdgeInterRobot(0, 2, 1, 5, fixed_weight))

        robot_id = 0
        nb_poses = 100
        nb_candidate_edges = 5
        max_nb_robots = 2
        nb_candidates_to_choose = 5
        use_vertex_cover = True

        params = {}
        params['robot_id'] = robot_id
        params['max_nb_robots'] = max_nb_robots
        params['frontend.similarity_threshold'] = 0.0  # Doesn't change anything
        params['frontend.sensor_type'] = 'stereo'
        params["frontend.enable_sparsification"] = True
        params["evaluation.enable_sparsification_comparison"] = False
        lcsm = LoopClosureSparseMatching(params)
        lcsm.candidate_selector.set_graph(fixed_edges_list, candidate_edges_list)

        # Solve the initial graph
        is_robot_considered = {}
        for i in range(max_nb_robots):
            is_robot_considered[i] = True
        selection = lcsm.select_candidates(nb_candidates_to_choose,
                                    is_robot_considered,
                                    greedy_initialization=False)

        self.assertEqual(
            len(selection), min(nb_candidate_edges, nb_candidates_to_choose))
        neighbors_in_range_list = range(max_nb_robots)

        # Brokerage
        broker = Broker(selection, neighbors_in_range_list)
        components = broker.brokerage(use_vertex_cover)

        nb_components = len(components)
        nb_vertices_transmitted = 0

        for component in components:
            for v in component:
                nb_vertices_transmitted += 1

        self.assertEqual(nb_components, 2)
        self.assertEqual(nb_vertices_transmitted, 2)




if __name__ == "__main__":
    unittest.main()
