import unittest
import sys  # TODO: remove

sys.path.append(
    '/home/lajoiepy/Documents/projects/c-slam/c-slam-ws/src/cslam/')
from cslam.algebraic_connectivity_maximization import AlgebraicConnectivityMaximization, EdgeInterRobot
from cslam.third_party.mac.utils import Edge
from cslam.third_party.mac.mac import MAC
import random
import numpy as np
from timeit import default_timer as timer


def build_simple_graph(nb_poses, nb_candidate_edges):
    # Build simple graph
    fixed_weight = 1
    fixed_edges_list = []

    candidate_edges = {}
    i = 0
    while len(candidate_edges.values()) < nb_candidate_edges:
        edge = EdgeInterRobot(0, random.choice(range(nb_poses)), 0,
                              random.choice(range(nb_poses)), fixed_weight)
        candidate_edges[(edge.robot0_image_id, edge.robot1_id)] = edge
        i = i + 1
    return fixed_edges_list, list(candidate_edges.values())


def build_multi_robot_graph(nb_poses, nb_candidate_edges, robot_id, nb_robots):
    # Build simple graph for local robot
    fixed_weight = 1
    fixed_edges_list = []

    # Enforce connectivity
    for i in range(nb_robots - 1):
        edge = EdgeInterRobot(i, nb_poses - 1, i + 1, nb_poses - 1,
                              fixed_weight)
        fixed_edges_list.append(edge)

    # Add inter-robot candidates
    candidate_edges = {}
    i = 0
    while len(candidate_edges.values()) < nb_candidate_edges:
        edge = EdgeInterRobot(random.choice(range(nb_robots)),
                              random.choice(range(nb_poses)),
                              random.choice(range(nb_robots)),
                              random.choice(range(nb_poses)), fixed_weight)
        candidate_edges[(edge.robot0_image_id, edge.robot1_id)] = edge
        i = i + 1
    return fixed_edges_list, list(candidate_edges.values())


class TestAlgebraicConnectivity(unittest.TestCase):
    """Unit tests for algebraic connectivity maximization
    """

    def test_simple_graph(self):
        """Test simple single robot graph
        """
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)

        nb_candidates_to_choose = 10

        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list)
        start = timer()
        selection = ac.select_candidates(nb_candidates_to_choose,
                                         greedy_initialization=False)
        stop = timer()
        self.assertEqual(len(selection), nb_candidates_to_choose)

    def test_greedy_initilization(self):
        """Test the greedy weight initialization
        """
        nb_candidates_to_choose = 10
        nb_poses = 100
        nb_candidate_edges = 50
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)
        weights = np.random.rand(nb_candidate_edges)
        i = 0
        for e in range(len(candidate_edges_list)):
            candidate_edges_list[e] = candidate_edges_list[e]._replace(
                weight=weights[i])
            i = i + 1
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list)
        is_robot_included = ac.check_graph_connectivity()
        ac.compute_offsets(is_robot_included)
        edges = ac.rekey_edges(ac.candidate_edges.values(), is_robot_included)
        w_init = ac.greedy_initialization(nb_candidates_to_choose, edges)
        self.assertAlmostEqual(
            np.sum(weights[w_init.astype(bool)]),
            np.sum(np.sort(weights)[-nb_candidates_to_choose:]))

    def test_add_measurements(self):
        """Test that adding measurements after solving works
        """
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        # Solve the initial graph
        selection0 = ac.select_candidates(nb_candidates_to_choose,
                                          greedy_initialization=False)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        # Add edges
        nb_add_edges = 10
        for i in range(nb_add_edges):
            ac.add_candidate_edge(
                EdgeInterRobot(0, random.choice(range(nb_poses)), 0,
                               random.choice(range(nb_poses)), 1.0))
        selection1 = ac.select_candidates(nb_candidates_to_choose,
                                          greedy_initialization=False)
        self.assertEqual(len(selection1), nb_candidates_to_choose)

        selection2 = ac.select_candidates(nb_candidates_to_choose + 2,
                                          greedy_initialization=False)
        self.assertEqual(len(selection2), nb_candidates_to_choose + 2)
        for i in range(nb_add_edges):
            ac.add_candidate_edge(
                EdgeInterRobot(0, random.choice(range(nb_poses)), 0,
                               random.choice(range(nb_poses)), 1.0))
        selection3 = ac.select_candidates(nb_candidates_to_choose + 2,
                                          greedy_initialization=False)
        self.assertEqual(len(selection3), nb_candidates_to_choose + 2)

    def test_fixed_loop_closures(self):
        """Test that loop closures can be fixed
        """
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        # Solve the initial graph
        selection0 = ac.select_candidates(nb_candidates_to_choose,
                                          greedy_initialization=False)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        # Add edges
        nb_add_edges = 10
        for i in range(nb_add_edges):
            ac.add_fixed_edge(
                EdgeInterRobot(0, random.choice(range(nb_poses)), 0,
                               random.choice(range(nb_poses)), 1.0))
        selection1 = ac.select_candidates(nb_candidates_to_choose,
                                          greedy_initialization=False)
        self.assertEqual(len(selection1), nb_candidates_to_choose)

    def test_remove_candidate(self):
        """Test that we can remove candidates from consideration
        """
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        # Solve the initial graph
        selection0 = ac.select_candidates(nb_candidates_to_choose,
                                          greedy_initialization=False)
        self.assertEqual(len(selection0), nb_candidates_to_choose)
        nb_candidates0 = len(ac.candidate_edges)

        # Remove edges
        ac.remove_candidate_edges(selection0)
        nb_candidates1 = len(ac.candidate_edges)
        self.assertEqual(nb_candidates0,
                         nb_candidates1 + nb_candidates_to_choose)

    def test_candidate_to_fixed(self):
        """Test that we can declare candidate fixed if they are successfully 
        computed
        """
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        # Solve the initial graph
        selection0 = ac.select_candidates(nb_candidates_to_choose,
                                          greedy_initialization=False)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        # Swap edge, make sure that none are the same
        ac.candidate_edges_to_fixed(selection0)
        selection1 = ac.select_candidates(nb_candidates_to_choose,
                                          greedy_initialization=False)
        for e0 in selection0:
            for e1 in selection1:
                self.assertFalse(e0.robot0_image_id == e1.robot0_image_id
                                 and e0.robot1_image_id == e1.robot1_image_id)

    def test_keys(self):
        """Test key changes between C-SLAM system and solver
        """
        robot_id = 0
        nb_poses = 10
        nb_candidate_edges = 10
        nb_robots = 3
        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, robot_id, nb_robots)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               nb_robots=nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        is_robot_included = ac.check_graph_connectivity()

        ac.compute_offsets(is_robot_included)
        rekeyed_fixed_edges = ac.rekey_edges(ac.fixed_edges, is_robot_included)
        self.assertEqual(len(ac.fixed_edges), 2)
        rekeyed_fixed_edges.extend(ac.fill_odometry())
        self.assertEqual(len(rekeyed_fixed_edges),
                         nb_robots * (nb_poses - 1) + 2)
        rekeyed_candidate_edges = ac.rekey_edges(ac.candidate_edges.values(), is_robot_included)
        values = list(ac.candidate_edges.values())
        for i in range(len(values)):
            e = values[i]
            r = rekeyed_candidate_edges[i]
            self.assertEqual(r.i, e.robot0_image_id + e.robot0_id * 10)
            self.assertEqual(r.j, e.robot1_image_id + e.robot1_id * 10)

        recovered_edges = ac.recover_inter_robot_edges(rekeyed_candidate_edges)
        for i in range(len(values)):
            e = values[i]
            r = recovered_edges[i]
            self.assertEqual(r.robot0_image_id, e.robot0_image_id)
            self.assertEqual(r.robot1_image_id, e.robot1_image_id)

    def test_multi_robot_edges0(self):
        """Test graph with multi-robot edges
        """
        robot_id = 0

        nb_poses = 100
        nb_candidate_edges = 100
        nb_robots = 3
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, robot_id, nb_robots)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               nb_robots=nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        # Solve the graph
        selection = ac.select_candidates(nb_candidates_to_choose,
                                         greedy_initialization=False)
        self.assertEqual(len(selection), nb_candidates_to_choose)
        for s in selection:
            self.assertLess(s.robot0_image_id, nb_poses)
            self.assertGreaterEqual(s.robot0_image_id, 0)
            self.assertLess(s.robot1_image_id, nb_poses)
            self.assertGreaterEqual(s.robot0_image_id, 0)
            self.assertGreaterEqual(s.robot0_id, 0)
            self.assertGreaterEqual(s.robot1_id, 0)
            self.assertLess(s.robot0_id, nb_robots)
            self.assertLess(s.robot1_id, nb_robots)

    def test_multi_robot_edges1(self):
        """Test graph with multi-robot edges
        """
        robot_id = 1

        nb_poses = 100
        nb_candidate_edges = 100
        nb_robots = 3
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, robot_id, nb_robots)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               nb_robots=nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        # Solve the graph
        selection = ac.select_candidates(nb_candidates_to_choose,
                                         greedy_initialization=False)
        self.assertEqual(len(selection), nb_candidates_to_choose)

    def test_add_match(self):
        """Test add match
        """
        robot_id = 0
        nb_robots = 3

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               nb_robots=nb_robots)

        e0 = EdgeInterRobot(0, 1, 1, 3, 0.1)
        ac.add_match(e0)
        self.assertEqual(len(ac.candidate_edges.values()), 1)
        e1 = EdgeInterRobot(0, 2, 2, 4, 0.1)
        ac.add_match(e1)
        self.assertEqual(len(ac.candidate_edges.values()), 2)
        e2 = EdgeInterRobot(0, 1, 1, 3, 0.2)
        ac.add_match(e2)
        self.assertEqual(len(ac.candidate_edges.values()), 2)
        self.assertAlmostEqual(
            ac.candidate_edges[(e2.robot0_image_id, e2.robot1_id)].weight, 0.2)


if __name__ == "__main__":
    unittest.main()
