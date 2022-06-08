import unittest
from cslam.algebraic_connectivity_maximization import AlgebraicConnectivityMaximization
from cslam.third_party.mac.utils import Edge
from cslam.third_party.mac.mac import MAC
import random
import numpy as np
from timeit import default_timer as timer


def build_simple_graph(nb_poses, nb_candidate_edges):
    # Build simple graph
    fixed_weight = 1
    fixed_edges_list = []
    for i in range(nb_poses - 1):
        edge = Edge(i, i + 1, fixed_weight)
        fixed_edges_list.append(edge)

    candidate_edges_list = []
    for i in range(nb_candidate_edges):
        edge = Edge(random.choice(range(nb_poses)),
                    random.choice(range(nb_poses)), fixed_weight)
        candidate_edges_list.append(edge)
    return fixed_edges_list, candidate_edges_list


class TestAlgebraicConnectivity(unittest.TestCase):
    """Unit tests for algebraic connectivity maximization
    """

    def test_simple_graph(self):
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)

        # Random Initialization
        mac = MAC(fixed_edges_list, candidate_edges_list, nb_poses)
        nb_candidates_to_choose = 10
        ac = AlgebraicConnectivityMaximization(
        )  # Use only for initialization here.
        ac.set_graph(fixed_edges_list, candidate_edges_list, nb_poses)
        ac.random_initialization(nb_candidates_to_choose)
        w_init = ac.w_init

        # Solve the relaxed maximum algebraic connectivity augmentation problem.
        start = timer()
        result, unrounded, upper = mac.fw_subset(w_init,
                                                 nb_candidates_to_choose,
                                                 max_iters=20)
        stop = timer()

        self.assertEqual(int(np.sum(result)), nb_candidates_to_choose)

        # Compare with the AlgebraicConnectivityMaximization wrapper
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list, nb_poses)
        start = timer()
        selection = ac.select_candidates(
            nb_candidates_to_choose)
        stop = timer()
        self.assertEqual(len(selection), nb_candidates_to_choose)

    def test_greedy_initilization(self):
        nb_candidates = 100
        nb_candidates_to_choose = 10
        weights = np.random.rand(nb_candidates)
        ac = AlgebraicConnectivityMaximization()
        ac.greedy_intialization(weights, nb_candidates_to_choose)
        self.assertAlmostEqual(
            np.sum(weights[ac.w_init.astype(bool)]),
            np.sum(np.sort(weights)[-nb_candidates_to_choose:]))

    def test_add_measurements(self):
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list, nb_poses)

        # Solve the initial graph
        selection0 = ac.select_candidates(
            nb_candidates_to_choose)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        # Add edges
        nb_add_edges = 10
        for i in range(nb_add_edges):
            ac.add_candidate_edge(
                Edge(random.choice(range(nb_poses)),
                     random.choice(range(nb_poses)), 1.0))
        selection1 = ac.select_candidates(
            nb_candidates_to_choose)
        self.assertEqual(len(selection1), nb_candidates_to_choose)

        selection2 = ac.select_candidates(
            nb_candidates_to_choose + 2)
        self.assertEqual(len(selection2), nb_candidates_to_choose + 2)
        for i in range(nb_add_edges):
            ac.add_candidate_edge(
                Edge(random.choice(range(nb_poses)),
                     random.choice(range(nb_poses)), 1.0))
        selection3 = ac.select_candidates(
            nb_candidates_to_choose + 2)
        self.assertEqual(len(selection3), nb_candidates_to_choose + 2)

    def test_fixed_loop_closures(self):
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list, nb_poses)

        # Solve the initial graph
        selection0 = ac.select_candidates(
            nb_candidates_to_choose)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        # Add edges
        nb_add_edges = 10
        for i in range(nb_add_edges):
            ac.add_fixed_edge(
                Edge(random.choice(range(nb_poses)),
                     random.choice(range(nb_poses)), 1.0))
        selection1 = ac.select_candidates(
            nb_candidates_to_choose)
        self.assertEqual(len(selection1), nb_candidates_to_choose)

    def test_remove_candidate(self):
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list, nb_poses)

        # Solve the initial graph
        selection0 = ac.select_candidates(
            nb_candidates_to_choose)
        self.assertEqual(len(selection0), nb_candidates_to_choose)
        nb_candidates0 = len(ac.candidate_edges)

        # Remove edges
        ac.remove_candidate_edges(selection0)
        nb_candidates1 = len(ac.candidate_edges)
        self.assertEqual(nb_candidates0,
                         nb_candidates1 + nb_candidates_to_choose)

    def test_candidate_to_fixed(self):
        # Build simple graph
        nb_poses = 100
        nb_candidate_edges = 50
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_simple_graph(
            nb_poses, nb_candidate_edges)
        ac = AlgebraicConnectivityMaximization()
        ac.set_graph(fixed_edges_list, candidate_edges_list, nb_poses)

        # Solve the initial graph
        selection0 = ac.select_candidates(
            nb_candidates_to_choose)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        # Swap edge, make sure that none are the same
        ac.candidate_edges_to_fixed(selection0)
        selection1 = ac.select_candidates(
            nb_candidates_to_choose)
        for e0 in selection0:
            for e1 in selection1:
                self.assertFalse(e0.i == e1.i and e0.j == e1.j)

    def test_multi_robot_edges(self):
        t = 0

if __name__ == "__main__":
    unittest.main()
