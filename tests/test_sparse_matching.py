import unittest
from cslam.loop_closure_sparse_matching import LoopClosureSparseMatching
import random
import numpy as np
from collections import namedtuple

GlobalDescriptor = namedtuple('GlobalDescriptor',
                              ['image_id', 'robot_id', 'descriptor'])


def set_params():
    params = {}
    params['robot_id'] = 0
    params['nb_robots'] = 2
    params['similarity_threshold'] = 0.0
    params['similarity_loc'] = 1.0
    params['similarity_scale'] = 0.25
    return params


class TestSparseMatching(unittest.TestCase):
    """Unit tests for sparse matching
    """

    def test_distance_to_similarity(self):
        """Distance to similarity
        """
        params = set_params()
        lcsm = LoopClosureSparseMatching(params)
        for i in range(0, 100, 1):
            d = i / 10
            s = lcsm.distance_to_similarity(d)
            self.assertGreaterEqual(s, 0.0)
            self.assertLessEqual(s, 1.0)

    def test_add_local_keyframe(self):
        """Add local keyframe
        """
        params = set_params()
        lcsm = LoopClosureSparseMatching(params)
        descriptor = np.random.rand(10)
        lcsm.add_local_keyframe(descriptor, 1)
        for i in range(len(descriptor)):
            self.assertAlmostEqual(descriptor[i], lcsm.local_nnsm.data[0, i])

    def test_add_other_robot_keyframe(self):
        """Add other robot keyframe
        """
        params = set_params()
        lcsm = LoopClosureSparseMatching(params)
        descriptor = np.random.rand(10)
        msg = GlobalDescriptor(0, 1, descriptor.tolist())
        lcsm.add_other_robot_keyframe(msg)
        for i in range(len(descriptor)):
            self.assertAlmostEqual(descriptor[i],
                                   lcsm.other_robots_nnsm[1].data[0, i])

    def test_matches(self):
        """Matches between descriptors
        """
        params = set_params()
        lcsm = LoopClosureSparseMatching(params)

        descriptor0 = np.random.rand(10)
        lcsm.add_local_keyframe(descriptor0, 2)

        descriptor1 = 1 - descriptor0
        msg = GlobalDescriptor(3, 1, descriptor1.tolist())
        lcsm.add_other_robot_keyframe(msg)

        descriptor2 = descriptor0
        descriptor2[0] = 0.0
        descriptor2[1] = 0.0
        msg2 = GlobalDescriptor(4, 1, descriptor2.tolist())
        lcsm.add_other_robot_keyframe(msg2)

        id = lcsm.candidate_selector.candidate_edges[(2, 1)].robot1_id
        best_match_descriptor = lcsm.other_robots_nnsm[id].data[0]
        for i in range(len(descriptor1)):
            self.assertAlmostEqual(descriptor1[i], best_match_descriptor[i])

    def test_select_candidates0(self):
        """Select candidates
        """
        params = set_params()
        params['nb_robots'] = 3
        lcsm = LoopClosureSparseMatching(params)

        nb_local_kfs = 100
        for i in range(nb_local_kfs):
            descriptor = np.random.rand(10)
            lcsm.add_local_keyframe(descriptor, i)
        nb_other_kfs = 100
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            msg = GlobalDescriptor(i, 1, descriptor.tolist())
            lcsm.add_other_robot_keyframe(msg)
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            msg = GlobalDescriptor(i, 2, descriptor.tolist())
            lcsm.add_other_robot_keyframe(msg)

        nb_candidates = 20
        selection = lcsm.select_candidates(nb_candidates)
        self.assertEqual(len(selection), nb_candidates)

    def test_select_candidates1(self):
        """Select candidates
            No robot 1 in range
        """
        params = set_params()
        params['nb_robots'] = 4
        lcsm = LoopClosureSparseMatching(params)

        nb_local_kfs = 100
        for i in range(nb_local_kfs):
            descriptor = np.random.rand(10)
            lcsm.add_local_keyframe(descriptor, i)
        nb_other_kfs = 100
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            msg = GlobalDescriptor(i, 2, descriptor.tolist())
            lcsm.add_other_robot_keyframe(msg)
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            msg = GlobalDescriptor(i, 3, descriptor.tolist())
            lcsm.add_other_robot_keyframe(msg)

        nb_candidates = 20
        selection = lcsm.select_candidates(nb_candidates)
        self.assertEqual(len(selection), nb_candidates)

    def test_select_candidates2(self):
        """Select candidates
            No robot 0 in range
        """
        params = set_params()
        params['nb_robots'] = 4
        params['robot_id'] = 1
        lcsm = LoopClosureSparseMatching(params)

        nb_local_kfs = 100
        for i in range(nb_local_kfs):
            descriptor = np.random.rand(10)
            lcsm.add_local_keyframe(descriptor, i)
        nb_other_kfs = 100
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            msg = GlobalDescriptor(i, 2, descriptor.tolist())
            lcsm.add_other_robot_keyframe(msg)
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            msg = GlobalDescriptor(i, 3, descriptor.tolist())
            lcsm.add_other_robot_keyframe(msg)

        nb_candidates = 20
        selection = lcsm.select_candidates(nb_candidates)
        self.assertEqual(len(selection), nb_candidates)

if __name__ == "__main__":
    unittest.main()
