import unittest

from cslam.broker import Broker
from cslam.algebraic_connectivity_maximization import AlgebraicConnectivityMaximization, EdgeInterRobot
from tests.test_algebraic_connectivity import build_multi_robot_graph
import random
import numpy as np
from collections import namedtuple
import math

def build_graph_and_extract_selection(nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidates_to_choose):
    """Build a graph and perform selection based on algebraic connectivity
    """
    fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, nb_robots)

    ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                            nb_robots=nb_robots)
    ac.set_graph(fixed_edges_list, candidate_edges_list)

    # Solve the initial graph
    is_robot_considered = {}
    for i in range(nb_robots):
        is_robot_considered[i] = True
    return ac.select_candidates(nb_candidates_to_choose, is_robot_considered,
                                        greedy_initialization=False)

def test_broker(unittest_framework, nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidates_to_choose, use_vertex_cover):
    """Test broker based on params
    """
    # Build graph
    selection = build_graph_and_extract_selection(nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidates_to_choose)
    unittest_framework.assertEqual(len(selection), nb_candidates_to_choose)
    neighbors_in_range_list = range(nb_robots)

    # Brokerage
    broker = Broker(selection, neighbors_in_range_list)
    components = broker.brokerage(use_vertex_cover)

    # Extract vertices
    initial_vertices = set()
    duplicates = []
    for e in selection:
        v0 = (e.robot0_id, e.robot0_image_id)
        if v0 in initial_vertices:
            duplicates.append(v0)
        initial_vertices.add(v0)
        v1 = (e.robot1_id, e.robot1_image_id)
        if v1 in initial_vertices:
            duplicates.append(v1)
        initial_vertices.add(v1)
    # Sanity check
    unittest_framework.assertEqual(len(initial_vertices) + len(duplicates), nb_candidates_to_choose*2)

    vertices = []
    for c in components:
        for v in c:
            vertices.append(v)

    # Check that we do not send more vertices than the trivial solution
    # Trivial solution: sending one vertex per edge
    # Upper Bounds:
    unittest_framework.assertLessEqual(len(vertices), nb_candidate_edges)
    if use_vertex_cover:
        unittest_framework.assertLessEqual(len(vertices), math.ceil(float(len(initial_vertices))/2))
    # Lower Bounds:
    unittest_framework.assertGreaterEqual(len(vertices), 1)

    # Check no duplicates
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                unittest_framework.assertNotEqual(vertices[i], vertices[j])

    # Check that all edges are covered
    for e in selection:
        v0 = (e.robot0_id, e.robot0_image_id)
        v1 = (e.robot1_id, e.robot1_image_id)
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
        nb_robots = 2
        nb_candidates_to_choose = 30
        use_vertex_cover = False

        test_broker(self, nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidates_to_choose, use_vertex_cover)

        test_broker(self, nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidate_edges, use_vertex_cover)

        test_broker(self, nb_poses*10, nb_candidate_edges*10, nb_robots, robot_id, nb_candidates_to_choose*10, use_vertex_cover)

        test_broker(self, nb_poses*10, nb_candidate_edges*10, nb_robots, robot_id, nb_candidate_edges*10, use_vertex_cover)
              

    def test_vertex_cover_2robots(self):
        """Vertex cover strategy
        """
        # Parameters
        robot_id = 0
        nb_poses = 100
        nb_candidate_edges = 50
        nb_robots = 2
        nb_candidates_to_choose = 30
        use_vertex_cover = True

        test_broker(self, nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidates_to_choose, use_vertex_cover)

        test_broker(self, nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidate_edges, use_vertex_cover)

        test_broker(self, nb_poses*10, nb_candidate_edges*10, nb_robots, robot_id, nb_candidates_to_choose*10, use_vertex_cover)

        test_broker(self, nb_poses*10, nb_candidate_edges*10, nb_robots, robot_id, nb_candidate_edges*10, use_vertex_cover)
           

    def test_simple_dialog_5robots(self):
        """Simple dialog strategy
        """
        # Parameters
        robot_id = 0
        nb_poses = 100
        nb_candidate_edges = 50
        nb_robots = 2
        nb_candidates_to_choose = 30
        use_vertex_cover = False

        test_broker(self, nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidates_to_choose, use_vertex_cover)

        test_broker(self, nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidate_edges, use_vertex_cover)

        test_broker(self, nb_poses*10, nb_candidate_edges*10, nb_robots, robot_id, nb_candidates_to_choose*10, use_vertex_cover)

        test_broker(self, nb_poses*10, nb_candidate_edges*10, nb_robots, robot_id, nb_candidate_edges*10, use_vertex_cover)
           

    def test_vertex_cover_5robots(self):
        """Vertex cover strategy
        """
        # Parameters
        robot_id = 0
        nb_poses = 100
        nb_candidate_edges = 50
        nb_robots = 2
        nb_candidates_to_choose = 30
        use_vertex_cover = True

        test_broker(self, nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidates_to_choose, use_vertex_cover)

        test_broker(self, nb_poses, nb_candidate_edges, nb_robots, robot_id, nb_candidate_edges, use_vertex_cover)

        test_broker(self, nb_poses*10, nb_candidate_edges*10, nb_robots, robot_id, nb_candidates_to_choose*10, use_vertex_cover)

        test_broker(self, nb_poses*10, nb_candidate_edges*10, nb_robots, robot_id, nb_candidate_edges*10, use_vertex_cover)
           

if __name__ == "__main__":
    unittest.main()
