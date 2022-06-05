from cslam.third_party.mac.mac import MAC
from cslam.third_party.mac.utils import Edge
import numpy as np


class AlgebraicConnectivityMaximization(object):

    def __init__(self, max_iters=20, fixed_weight=1.0):
        self.fixed_weight = fixed_weight
        self.fixed_edges = []
        self.candidate_edges = []
        self.nb_poses = 0
        self.max_iters = max_iters

    def set_graph(self, fixed_edges, candidate_edges, nb_poses):
        self.fixed_edges = fixed_edges
        self.candidate_edges = candidate_edges
        self.nb_poses = nb_poses

    def add_fixed_edge(self, edge):
        self.fixed_edges.append(edge)

    def add_candidate_edge(self, edge):
        self.candidate_edges.append(edge)

    def remove_candidate_edges(self, edges):
        self.candidate_edges = [
            self.candidate_edges[i] for i in range(len(self.candidate_edges))
            if (self.candidate_edges[i] not in edges)
        ]

    def candidate_edges_to_fixed(self, edges):
        self.fixed_edges.extend(edges)
        self.remove_candidate_edges(edges)

    def greedy_intialization(self, scores, nb_candidates_to_choose):
        self.w_init = np.zeros(len(scores))
        indices = np.argpartition(
            scores, -nb_candidates_to_choose)[-nb_candidates_to_choose:]
        self.w_init[indices] = 1.0

    def random_initialization(self, nb_candidates_to_choose):
        scores = np.random.rand(len(self.candidate_edges))
        self.greedy_intialization(scores, nb_candidates_to_choose)

    def select_candidates(self, nb_candidates_to_choose):
        self.mac = MAC(self.fixed_edges, self.candidate_edges, self.nb_poses)
        self.random_initialization(nb_candidates_to_choose)
        result, u, _ = self.mac.fw_subset(self.w_init,
                                          nb_candidates_to_choose,
                                          max_iters=self.max_iters)
        return [
            self.candidate_edges[i] for i in np.nonzero(result.astype(int))[0]
        ]
