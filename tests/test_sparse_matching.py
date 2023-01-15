import unittest

from cslam.loop_closure_sparse_matching import LoopClosureSparseMatching
from cslam.nns_matching import NearestNeighborsMatching
import random
import numpy as np
from collections import namedtuple

GlobalDescriptor = namedtuple('GlobalDescriptor',
                              ['keyframe_id', 'robot_id', 'descriptor'])

def set_params():
    params = {}
    params['robot_id'] = 0
    params['max_nb_robots'] = 2
    params['frontend.sensor_type'] = 'stereo'
    params['frontend.similarity_threshold'] = 0.0
    params["frontend.enable_sparsification"] = True
    params["evaluation.enable_sparsification_comparison"] = False
    return params


class TestSparseMatching(unittest.TestCase):
    """Unit tests for sparse matching
    """

    def test_add_local_global_descriptor(self):
        """Add local keyframe
        """
        params = set_params()
        lcsm = LoopClosureSparseMatching(params)
        descriptor = np.random.rand(10)
        descriptor = descriptor / np.linalg.norm(descriptor)  
        lcsm.add_local_global_descriptor(descriptor, 1)
        for i in range(len(descriptor)):
            self.assertAlmostEqual(descriptor[i], lcsm.local_nnsm.data[0, i])

    def test_add_other_robot_global_descriptor(self):
        """Add other robot keyframe
        """
        params = set_params()
        lcsm = LoopClosureSparseMatching(params)
        descriptor = np.random.rand(10)
        descriptor = descriptor / np.linalg.norm(descriptor)  
        msg = GlobalDescriptor(0, 1, descriptor.tolist())
        lcsm.add_other_robot_global_descriptor(msg)
        for i in range(len(descriptor)):
            self.assertAlmostEqual(descriptor[i],
                                   lcsm.other_robots_nnsm[1].data[0, i])

    def test_similarity(self):
        """Test that cosine similarity matching gives the same ordering as euclidean distance
        """
        nb_descriptors_in_db = 100
        nnsm = NearestNeighborsMatching()
        for i in range(nb_descriptors_in_db):
            descriptor = np.random.rand(100)
            descriptor = descriptor / np.linalg.norm(descriptor) # normalize
            nnsm.add_item(descriptor, i)

        nb_descriptors_to_test = 100
        k = 100
        for i in range(nb_descriptors_to_test):
            query = np.random.rand(100)
            query = query / np.linalg.norm(query)
            ds = np.linalg.norm(query[np.newaxis, :] - nnsm.data[:nnsm.n], axis=1)
            ns_dist = np.argsort(ds)[:k]
            ns_sim, sims = nnsm.search(query, k)
            # Check if sorted by similarity
            self.assertTrue(np.all(sims[:-1] >= sims[1:]))
            # Check if same order as distance
            for j in range(k):
                id_d = ns_dist[j]
                id_s = ns_sim[j]
                if (id_d != id_s):
                    # If the distance or similarity are the same than the order does not matter.
                    if (abs(sims[id_d] - sims[id_s]) < 1e-6) or (abs(ds[id_d] - ds[id_s]) < 1e-6):
                        continue
                self.assertEqual(ns_dist[j], ns_sim[j])
            ns_sim_best, sim_best = nnsm.search_best(query)     
            self.assertEqual(ns_dist[0], ns_sim_best)   

    def test_matches(self):
        """Matches between descriptors
        """
        params = set_params()
        lcsm = LoopClosureSparseMatching(params)

        descriptor0 = np.random.rand(10)
        descriptor0 = descriptor0 / np.linalg.norm(descriptor0)        
        lcsm.add_local_global_descriptor(descriptor0, 2)

        descriptor1 = 1 - descriptor0
        descriptor1 = descriptor1 / np.linalg.norm(descriptor1)   
        msg = GlobalDescriptor(3, 1, descriptor1.tolist())
        lcsm.add_other_robot_global_descriptor(msg)

        descriptor2 = descriptor0
        descriptor2[0] = 0.0
        descriptor2[1] = 0.0
        descriptor2 = descriptor2 / np.linalg.norm(descriptor2)  
        msg2 = GlobalDescriptor(4, 1, descriptor2.tolist())
        lcsm.add_other_robot_global_descriptor(msg2)

        id = lcsm.candidate_selector.candidate_edges[(0, 2, 1, 4)].robot1_id
        best_match_descriptor = lcsm.other_robots_nnsm[id].data[0]
        for i in range(len(descriptor1)):
            self.assertAlmostEqual(descriptor1[i], best_match_descriptor[i])

    def test_select_candidates0(self):
        """Select candidates
        """
        params = set_params()
        params['max_nb_robots'] = 3
        lcsm = LoopClosureSparseMatching(params)

        nb_local_kfs = 100
        for i in range(nb_local_kfs):
            descriptor = np.random.rand(10)
            descriptor = descriptor / np.linalg.norm(descriptor)  
            lcsm.add_local_global_descriptor(descriptor, i)
        nb_other_kfs = 100
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            descriptor = descriptor / np.linalg.norm(descriptor)  
            msg = GlobalDescriptor(i, 1, descriptor.tolist())
            lcsm.add_other_robot_global_descriptor(msg)
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            descriptor = descriptor / np.linalg.norm(descriptor)  
            msg = GlobalDescriptor(i, 2, descriptor.tolist())
            lcsm.add_other_robot_global_descriptor(msg)

        nb_candidates = 20
        is_robot_considered = {}
        for i in range(params['max_nb_robots']):
            is_robot_considered[i] = True
        selection = lcsm.select_candidates(nb_candidates, is_robot_considered)
        self.assertEqual(len(selection), nb_candidates)

    def test_select_candidates1(self):
        """Select candidates
            No robot 1 in range
        """
        params = set_params()
        params['max_nb_robots'] = 4
        lcsm = LoopClosureSparseMatching(params)

        nb_local_kfs = 100
        for i in range(nb_local_kfs):
            descriptor = np.random.rand(10)
            descriptor = descriptor / np.linalg.norm(descriptor)  
            lcsm.add_local_global_descriptor(descriptor, i)
        nb_other_kfs = 100
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            descriptor = descriptor / np.linalg.norm(descriptor)  
            msg = GlobalDescriptor(i, 2, descriptor.tolist())
            lcsm.add_other_robot_global_descriptor(msg)
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            descriptor = descriptor / np.linalg.norm(descriptor)  
            msg = GlobalDescriptor(i, 3, descriptor.tolist())
            lcsm.add_other_robot_global_descriptor(msg)

        nb_candidates = 20

        is_robot_considered = {}
        for i in range(params['max_nb_robots']):
            is_robot_considered[i] = True
        selection = lcsm.select_candidates(nb_candidates, is_robot_considered)
        self.assertEqual(len(selection), nb_candidates)

    def test_select_candidates2(self):
        """Select candidates
            No robot 0 in range
        """
        params = set_params()
        params['max_nb_robots'] = 4
        params['robot_id'] = 1
        lcsm = LoopClosureSparseMatching(params)

        nb_local_kfs = 100
        for i in range(nb_local_kfs):
            descriptor = np.random.rand(10)
            descriptor = descriptor / np.linalg.norm(descriptor)  
            lcsm.add_local_global_descriptor(descriptor, i)
        nb_other_kfs = 100
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            descriptor = descriptor / np.linalg.norm(descriptor)  
            msg = GlobalDescriptor(i, 2, descriptor.tolist())
            lcsm.add_other_robot_global_descriptor(msg)
        for i in range(nb_other_kfs):
            descriptor = np.random.rand(10)
            descriptor = descriptor / np.linalg.norm(descriptor)  
            msg = GlobalDescriptor(i, 3, descriptor.tolist())
            lcsm.add_other_robot_global_descriptor(msg)

        nb_candidates = 20

        is_robot_considered = {}
        for i in range(params['max_nb_robots']):
            is_robot_considered[i] = True

        selection = lcsm.select_candidates(nb_candidates, is_robot_considered)
        self.assertEqual(len(selection), nb_candidates)


if __name__ == "__main__":
    unittest.main()
