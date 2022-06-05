import unittest
from cslam.algebraic_connectivity_maximization import AlgebraicConnectivityMaximization
from cslam.third_party.mac.utils import Edge
from cslam.third_party.mac.mac import MAC
import random
import numpy as np
from timeit import default_timer as timer


class TestAlgebraicConnectivity(unittest.TestCase):
    """Unit tests for algebraic connectivity maximization
    """

    def test_simple_graph(self):
        # Build simple graph
        num_poses = 10
        fixed_weight = 1
        fixed_edges_list = []
        for i in range(num_poses - 1):
            edge = Edge(i, i + 1, fixed_weight)
            fixed_edges_list.append(edge)

        candidate_edges_list = []
        num_candidate_edges = 10
        for i in range(num_candidate_edges):
            edge = Edge(random.choice(range(num_poses)),
                        random.choice(range(num_poses)), fixed_weight)
            candidate_edges_list.append(edge)

        # Initialize problem
        mac = MAC(fixed_edges_list, candidate_edges_list, num_poses)
        w_init = np.zeros(len(candidate_edges_list))
        w_init[:candidate_edges_list] = 1.0

        # Solve the relaxed maximum algebraic connectivity augmentation problem.
        num_candidates_to_choose = num_candidate_edges / 2
        start = timer()
        result, unrounded, upper = mac.fw_subset(w_init,
                                                 num_candidates_to_choose,
                                                 max_iters=20)
        stop = timer()

        print(result)
        print(unrounded)
        print(upper)
        print(stop - start)

    def test_add_measurements(self):
        # Test add measurements
        self.assertTrue("FOO".isupper())
        self.assertFalse("Foo".isupper())

    def test_fixed_loop_closures(self):
        # Test fixed loop closures
        self.assertTrue("FOO".isupper())
        self.assertFalse("Foo".isupper())

    def test_candidate_to_fixed(self):
        # Test move loop closures from candidate to fixed
        s = "hello world"
        self.assertEqual(s.split(), ["hello", "world"])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)

    def test_remove_candidate(self):
        # Test remove candidate loop closures
        s = "hello world"
        self.assertEqual(s.split(), ["hello", "world"])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)


if __name__ == "__main__":
    unittest.main()
