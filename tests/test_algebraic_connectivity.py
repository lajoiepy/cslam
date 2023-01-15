import unittest

import sys
sys.path.append('/home/lajoiepy/Documents/projects/c-slam/c-slam-ws/install/cslam/lib/python3.8/site-packages/')

from cslam.algebraic_connectivity_maximization import AlgebraicConnectivityMaximization, EdgeInterRobot
from cslam.mac.utils import Edge
from cslam.mac.mac import MAC
import random
import numpy as np
from timeit import default_timer as timer
import copy


def build_simple_graph(nb_poses, nb_candidate_edges):
    """Build simple graph

    Args:
        nb_poses (int): nb of poses in graph
        nb_candidate_edges (int): nb of loop edges to generate

    Returns:
        list(EdgeInterRobot), dict(EdgeInterRobot): edges in the graph
    """
    fixed_weight = 1
    fixed_edges_list = []

    candidate_edges = {}
    i = 0
    while len(candidate_edges.values()) < nb_candidate_edges:
        edge = EdgeInterRobot(0, random.choice(range(nb_poses)), 0,
                              random.choice(range(nb_poses)), fixed_weight)
        candidate_edges[(edge.robot0_keyframe_id, edge.robot1_keyframe_id)] = edge
        i = i + 1
    return fixed_edges_list, list(candidate_edges.values())


def build_multi_robot_graph(nb_poses, nb_candidate_edges, max_nb_robots):
    """Build graph with multiple robots

    Args:
        nb_poses (int): nb of poses in graph
        nb_candidate_edges (int): nb of loop edges to generate
        max_nb_robots (int): nb of robots

    Returns:
        list(EdgeInterRobot), dict(EdgeInterRobot): edges in the graph
    """
    fixed_weight = 1
    fixed_edges_list = []

    # Enforce connectivity
    for i in range(max_nb_robots - 1):
        edge = EdgeInterRobot(i, nb_poses - 1, i + 1, nb_poses - 1,
                              fixed_weight)
        fixed_edges_list.append(edge)

    # Add inter-robot candidates
    candidate_edges = {}
    i = 0
    while len(candidate_edges.values()) < nb_candidate_edges:
        robot0_id = random.choice(range(max_nb_robots))
        robot1_id = random.choice(
            list(set(range(max_nb_robots)) - set([robot0_id])))
        edge = EdgeInterRobot(robot0_id,
                              random.choice(range(nb_poses)), robot1_id,
                              random.choice(range(nb_poses)), fixed_weight)
        if edge.robot0_id < edge.robot1_id:                      
            candidate_edges[(edge.robot0_id, edge.robot0_keyframe_id, edge.robot1_id, edge.robot1_keyframe_id)] = edge
        else:
            candidate_edges[(edge.robot1_id, edge.robot1_keyframe_id, edge.robot0_id, edge.robot0_keyframe_id)] = edge
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
        is_robot_considered = {0: True}
        selection = ac.select_candidates(nb_candidates_to_choose,
                                         is_robot_considered,
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
        ac = AlgebraicConnectivityMaximization()
        i = 0
        for e in range(len(candidate_edges_list)):
            candidate_edges_list[e] = ac.replace_weight(candidate_edges_list[e],
                weight=weights[i])
            i = i + 1
        ac.set_graph(fixed_edges_list, candidate_edges_list)
        is_robot_considered = {0: True}
        is_robot_included = ac.check_graph_disconnections(is_robot_considered)
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
        is_robot_considered = {0: True}
        selection0 = ac.select_candidates(nb_candidates_to_choose,
                                          is_robot_considered,
                                          greedy_initialization=False)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        # Add edges
        nb_add_edges = 10
        for i in range(nb_add_edges):
            ac.add_candidate_edge(
                EdgeInterRobot(0, random.choice(range(nb_poses)), 0,
                               random.choice(range(nb_poses)), 1.0))
        selection1 = ac.select_candidates(nb_candidates_to_choose,
                                          is_robot_considered,
                                          greedy_initialization=False)
        self.assertEqual(len(selection1), nb_candidates_to_choose)

        selection2 = ac.select_candidates(nb_candidates_to_choose + 2,
                                          is_robot_considered,
                                          greedy_initialization=False)
        self.assertEqual(len(selection2), nb_candidates_to_choose + 2)
        for i in range(nb_add_edges):
            ac.add_candidate_edge(
                EdgeInterRobot(0, random.choice(range(nb_poses)), 0,
                               random.choice(range(nb_poses)), 1.0))
        selection3 = ac.select_candidates(nb_candidates_to_choose + 2,
                                          is_robot_considered,
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
        is_robot_considered = {0: True}
        selection0 = ac.select_candidates(nb_candidates_to_choose,
                                          is_robot_considered,
                                          greedy_initialization=False)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        # Add edges
        nb_add_edges = 10
        for i in range(nb_add_edges):
            ac.add_fixed_edge(
                EdgeInterRobot(0, random.choice(range(nb_poses)), 0,
                               random.choice(range(nb_poses)), 1.0))
        selection1 = ac.select_candidates(nb_candidates_to_choose,
                                          is_robot_considered,
                                          greedy_initialization=False)
        self.assertEqual(len(selection1), nb_candidates_to_choose)

    def test_remove_candidate0(self):
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
        candidates_edges_before = ac.candidate_edges.copy()
        is_robot_considered = {0: True}
        selection0 = ac.select_candidates(nb_candidates_to_choose,
                                          is_robot_considered,
                                          greedy_initialization=False)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        test = set()
        for e in selection0:
            self.assertIn(e, list(candidates_edges_before.values()))
            test.add(e)
        # Check that there are no duplicates
        self.assertEqual(len(test), nb_candidates_to_choose)

        nb_candidates0 = len(ac.candidate_edges)

        # Remove edges
        ac.remove_candidate_edges(list(ac.candidate_edges.values())[:nb_candidates_to_choose])
        nb_candidates1 = len(ac.candidate_edges)
        self.assertGreaterEqual(nb_candidates0,
                                nb_candidates1 + nb_candidates_to_choose - 1)
        self.assertLessEqual(nb_candidates0,
                             nb_candidates1 + nb_candidates_to_choose + 1)

    def test_remove_candidate1(self):
        """Test that we can remove candidates from consideration
        """
        # Build graph
        robot_id = 0
        nb_poses = 10
        nb_candidate_edges = 10
        max_nb_robots = 3
        nb_candidates_to_choose = 3
        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, max_nb_robots)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)
        candidates_edges_before = ac.candidate_edges.copy()

        # Solve the initial graph
        is_robot_considered = {}
        for i in range(max_nb_robots):
            is_robot_considered[i] = True
        selection0 = ac.select_candidates(nb_candidates_to_choose,
                                          is_robot_considered,
                                          greedy_initialization=False)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        for e in selection0:
            self.assertIn(e, list(candidates_edges_before.values()))

        nb_candidates0 = len(ac.candidate_edges)

        # Remove edges
        existant_edge = [list(ac.candidate_edges.values())[0]]
        inexistant_edge = [EdgeInterRobot(0, 1, 4, 1, 1.0)]
        ac.remove_candidate_edges(existant_edge)
        nb_candidates1 = len(ac.candidate_edges)
        self.assertEqual(nb_candidates0, nb_candidates1 + 1)

        ac.remove_candidate_edges(inexistant_edge)
        nb_candidates2 = len(ac.candidate_edges)
        self.assertEqual(nb_candidates0, nb_candidates2 + 1)

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

        candidates_edges_before = ac.candidate_edges.copy()

        # Solve the initial graph
        is_robot_considered = {0: True}
        selection0 = ac.select_candidates(nb_candidates_to_choose,
                                          is_robot_considered,
                                          greedy_initialization=False)
        self.assertEqual(len(selection0), nb_candidates_to_choose)

        for e in selection0:
            self.assertIn(e, list(candidates_edges_before.values()))

        # Swap edge, make sure that none are the same
        ac.candidate_edges_to_fixed(selection0)
        for e in selection0:
            self.assertNotIn(e, list(ac.candidate_edges.values()))

        selection1 = ac.select_candidates(nb_candidates_to_choose,
                                          is_robot_considered,
                                          greedy_initialization=False)

        for e in selection1:
            self.assertIn(e, list(candidates_edges_before.values()))

        for e0 in selection0:
            for e1 in selection1:
                self.assertFalse(e0.robot0_keyframe_id == e1.robot0_keyframe_id
                                 and e0.robot1_keyframe_id == e1.robot1_keyframe_id)

    def test_check_graph_disconnections(self):
        """Test connectivity check
        """
        # All robots connected
        robot_id = 0
        nb_poses = 10
        nb_candidate_edges = 10
        max_nb_robots = 3
        is_robot_considered = {}
        for i in range(max_nb_robots):
            is_robot_considered[i] = True

        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, max_nb_robots)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        is_robot_included = ac.check_graph_disconnections(is_robot_considered)

        for r in is_robot_included:
            self.assertTrue(is_robot_included[r])

        is_robot_considered[1] = False

        is_robot_included = ac.check_graph_disconnections(is_robot_considered)
        for r in is_robot_included:
            if is_robot_considered[r]:
                self.assertTrue(is_robot_included[r])
            else:
                self.assertFalse(is_robot_included[r])

        is_robot_considered[1] = True

        # Robot 0 not connected
        robot_id = 1
        nb_poses = 10
        nb_candidate_edges = 10
        max_nb_robots = 3
        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, max_nb_robots)

        to_delete = []
        for i in range(len(fixed_edges_list)):
            if fixed_edges_list[i].robot0_id == 0 or fixed_edges_list[
                    i].robot1_id == 0:
                to_delete.append(fixed_edges_list[i])
        for e in to_delete:
            fixed_edges_list.remove(e)

        to_delete = []
        for i in range(len(candidate_edges_list)):
            if candidate_edges_list[i].robot0_id == 0 or candidate_edges_list[
                    i].robot1_id == 0:
                to_delete.append(candidate_edges_list[i])
        for e in to_delete:
            candidate_edges_list.remove(e)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        is_robot_included = ac.check_graph_disconnections(is_robot_considered)

        for r in is_robot_included:
            if r == 0:
                self.assertFalse(is_robot_included[r])
            else:
                self.assertTrue(is_robot_included[r])

    def test_compute_offsets(self):
        """Test connectivity check
        """
        # All robots connected
        robot_id = 1
        nb_poses = 10
        nb_candidate_edges = 10
        max_nb_robots = 5
        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, max_nb_robots)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        is_robot_considered = {}
        for i in range(max_nb_robots):
            is_robot_considered[i] = True
        is_robot_included = ac.check_graph_disconnections(is_robot_considered)
        ac.compute_offsets(is_robot_included)

        nb_poses = ac.nb_poses
        self.assertEqual(ac.offsets[0], 0)
        self.assertEqual(ac.offsets[1], ac.offsets[0] + nb_poses[0])
        self.assertEqual(ac.offsets[2], ac.offsets[1] + nb_poses[1])
        self.assertEqual(ac.offsets[3], ac.offsets[2] + nb_poses[2])
        self.assertEqual(ac.offsets[4], ac.offsets[3] + nb_poses[3])

        # Robot 0 not connected
        to_delete = []
        for i in range(len(fixed_edges_list)):
            if fixed_edges_list[i].robot0_id == 0 or fixed_edges_list[
                    i].robot1_id == 0:
                to_delete.append(fixed_edges_list[i])
        for e in to_delete:
            fixed_edges_list.remove(e)

        to_delete = []
        for i in range(len(candidate_edges_list)):
            if candidate_edges_list[i].robot0_id == 0 or candidate_edges_list[
                    i].robot1_id == 0:
                to_delete.append(candidate_edges_list[i])
        for e in to_delete:
            candidate_edges_list.remove(e)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        is_robot_included = ac.check_graph_disconnections(is_robot_considered)
        ac.compute_offsets(is_robot_included)

        self.assertEqual(ac.offsets[0], 0)
        self.assertEqual(ac.offsets[1], 0)
        self.assertEqual(ac.offsets[2], ac.offsets[1] + nb_poses[1])
        self.assertEqual(ac.offsets[3], ac.offsets[2] + nb_poses[2])
        self.assertEqual(ac.offsets[4], ac.offsets[3] + nb_poses[3])

        # Robot 0 and 3 not connected
        to_delete = []
        for i in range(len(fixed_edges_list)):
            if fixed_edges_list[i].robot0_id == 3 or fixed_edges_list[
                    i].robot1_id == 3:
                to_delete.append(fixed_edges_list[i])
        for e in to_delete:
            fixed_edges_list.remove(e)

        to_delete = []
        for i in range(len(candidate_edges_list)):
            if candidate_edges_list[i].robot0_id == 3 or candidate_edges_list[
                    i].robot1_id == 3:
                to_delete.append(candidate_edges_list[i])
        for e in to_delete:
            candidate_edges_list.remove(e)

        # Ensure connectivity
        edge = EdgeInterRobot(1, 1, 4, 1, 1.0)
        fixed_edges_list.append(edge)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        is_robot_included = ac.check_graph_disconnections(is_robot_considered)
        ac.compute_offsets(is_robot_included)

        self.assertEqual(ac.offsets[0], 0)
        self.assertEqual(ac.offsets[1], 0)
        self.assertEqual(ac.offsets[2], ac.offsets[1] + nb_poses[1])
        self.assertEqual(ac.offsets[3], 0)
        self.assertEqual(ac.offsets[4], ac.offsets[2] + nb_poses[2])

    def test_keys(self):
        """Test key changes between C-SLAM system and solver
        """
        robot_id = 0
        nb_poses = 10
        nb_candidate_edges = 10
        max_nb_robots = 3
        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, max_nb_robots)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        is_robot_considered = {}
        for i in range(max_nb_robots):
            is_robot_considered[i] = True
        is_robot_included = ac.check_graph_disconnections(is_robot_considered)

        ac.compute_offsets(is_robot_included)
        rekeyed_fixed_edges = ac.rekey_edges(ac.fixed_edges, is_robot_included)
        self.assertEqual(len(ac.fixed_edges), 2)
        rekeyed_fixed_edges.extend(ac.fill_odometry())
        self.assertEqual(len(rekeyed_fixed_edges),
                         max_nb_robots * (nb_poses - 1) + 2)
        rekeyed_candidate_edges = ac.rekey_edges(ac.candidate_edges.values(),
                                                 is_robot_included)
        values = list(ac.candidate_edges.values())
        for i in range(len(values)):
            e = values[i]
            r = rekeyed_candidate_edges[i]
            self.assertEqual(r.i, e.robot0_keyframe_id + e.robot0_id * 10)
            self.assertEqual(r.j, e.robot1_keyframe_id + e.robot1_id * 10)

        recovered_edges = ac.recover_inter_robot_edges(rekeyed_candidate_edges,
                                                       is_robot_included)
        for i in range(len(values)):
            e = values[i]
            r = recovered_edges[i]
            self.assertEqual(r.robot0_keyframe_id, e.robot0_keyframe_id)
            self.assertEqual(r.robot1_keyframe_id, e.robot1_keyframe_id)

    def test_multi_robot_edges0(self):
        """Test graph with multi-robot edges
        """
        robot_id = 0

        nb_poses = 100
        nb_candidate_edges = 100
        max_nb_robots = 3
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, max_nb_robots)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        is_robot_considered = {}
        for i in range(max_nb_robots):
            is_robot_considered[i] = True

        # Solve the graph
        selection = ac.select_candidates(nb_candidates_to_choose,
                                         is_robot_considered,
                                         greedy_initialization=False)
        self.assertEqual(len(selection), nb_candidates_to_choose)
        for s in selection:
            self.assertLess(s.robot0_keyframe_id, nb_poses)
            self.assertGreaterEqual(s.robot0_keyframe_id, 0)
            self.assertLess(s.robot1_keyframe_id, nb_poses)
            self.assertGreaterEqual(s.robot0_keyframe_id, 0)
            self.assertGreaterEqual(s.robot0_id, 0)
            self.assertGreaterEqual(s.robot1_id, 0)
            self.assertLess(s.robot0_id, max_nb_robots)
            self.assertLess(s.robot1_id, max_nb_robots)

    def test_multi_robot_edges1(self):
        """Test graph with multi-robot edges
        """
        robot_id = 1

        nb_poses = 100
        nb_candidate_edges = 100
        max_nb_robots = 3
        nb_candidates_to_choose = 10
        fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
            nb_poses, nb_candidate_edges, max_nb_robots)

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)
        ac.set_graph(fixed_edges_list, candidate_edges_list)

        is_robot_considered = {}
        for i in range(max_nb_robots):
            is_robot_considered[i] = True

        # Solve the graph
        selection = ac.select_candidates(nb_candidates_to_choose,
                                         is_robot_considered,
                                         greedy_initialization=False)
        self.assertEqual(len(selection), nb_candidates_to_choose)

    def test_result_different_than_initial_guess(self):
        """Test that the initial guess and results are not the same
        """
        robot_id = 0

        nb_poses = 100
        nb_candidate_edges = 100
        max_nb_robots = 3
        nb_candidates_to_choose = 10

        is_robot_considered = {}
        for i in range(max_nb_robots):
            is_robot_considered[i] = True

        # Should at least differ half the time
        nb_tests = 10
        nb_diff = 0
        for i in range(nb_tests):
            fixed_edges_list, candidate_edges_list = build_multi_robot_graph(
                nb_poses, nb_candidate_edges, max_nb_robots)

            ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                                   max_nb_robots=max_nb_robots)
            ac.set_graph(fixed_edges_list, candidate_edges_list)

            # Check that the solution differs from the greedy initial guess
            is_robot_included = ac.check_graph_disconnections(
                is_robot_considered)
            ac.compute_offsets(is_robot_included)
            rekeyed_fixed_edges = ac.rekey_edges(ac.fixed_edges,
                                                 is_robot_included)
            rekeyed_fixed_edges.extend(ac.fill_odometry())
            rekeyed_candidate_edges = ac.rekey_edges(
                ac.candidate_edges.values(), is_robot_included)

            # Compute number of poses
            ac.total_nb_poses = 0
            for n in range(len(ac.nb_poses)):
                ac.total_nb_poses = ac.total_nb_poses + ac.nb_poses[n]

            # Initial guess
            w_init = ac.greedy_initialization(nb_candidates_to_choose,
                                              rekeyed_candidate_edges)

            result = ac.run_mac_solver(rekeyed_fixed_edges,
                                       rekeyed_candidate_edges, w_init,
                                       nb_candidates_to_choose)

            is_same = True
            for i in range(len(w_init)):
                if abs(w_init[i] - result[i]) > 0.5:
                    is_same = False

            if not is_same:
                nb_diff = nb_diff + 1

        self.assertGreaterEqual(nb_diff, int(nb_tests / 2))

    def test_add_match(self):
        """Test add match
        """
        robot_id = 0
        max_nb_robots = 3

        ac = AlgebraicConnectivityMaximization(robot_id=robot_id,
                                               max_nb_robots=max_nb_robots)

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
            ac.candidate_edges[(e2.robot0_id, e2.robot0_keyframe_id, e2.robot1_id,
                                e2.robot1_keyframe_id)].weight, 0.2)


if __name__ == "__main__":
    unittest.main()
