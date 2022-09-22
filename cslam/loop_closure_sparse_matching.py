import numpy as np
from cslam.nns_matching import NearestNeighborsMatching
from cslam.algebraic_connectivity_maximization import AlgebraicConnectivityMaximization, EdgeInterRobot


class LoopClosureSparseMatching(object):
    """Sparse matching for loop closure detection
        Matches global descriptors to generate loop closure candidates
        Then candidates are selected such that we respect the communication budget
    """

    def __init__(self, params):
        """ Initialization of loop closure matching

        Args:
            params (dict): ROS 2 parameters
        """
        # Extract params
        self.params = params
        # Initialize matching structs
        self.local_nnsm = NearestNeighborsMatching()
        self.other_robots_nnsm = {}
        for i in range(self.params['nb_robots']):
            if i != self.params['robot_id']:
                self.other_robots_nnsm[i] = NearestNeighborsMatching()
        # Initialize candidate selection algorithm
        self.candidate_selector = AlgebraicConnectivityMaximization(
            self.params['robot_id'], self.params['nb_robots'])

    def add_local_global_descriptor(self, embedding, id):
        """ Add a local keyframe for matching

        Args:
            embedding (np.array): global descriptor
            id (int): keyframe id
        """
        self.local_nnsm.add_item(embedding, id)
        for i in range(self.params['nb_robots']):
            if i != self.params['robot_id']:
                kf, similarity = self.other_robots_nnsm[i].search_best(embedding)
                if kf is not None:
                    if similarity >= self.params['frontend.similarity_threshold']:
                        self.candidate_selector.add_match(
                            EdgeInterRobot(self.params['robot_id'], id, i, kf,
                                           similarity))

    def add_other_robot_global_descriptor(self, msg):
        """ Add keyframe global descriptor info from other robot

        Args:
            msg (cslam_loop_detection_interfaces.msg.GlobalImageDescriptor): global descriptor info
        """
        self.other_robots_nnsm[msg.robot_id].add_item(
            np.asarray(msg.descriptor), msg.image_id)

        kf, similarity = self.local_nnsm.search_best(np.asarray(msg.descriptor))
        if kf is not None:
            if similarity >= self.params['frontend.similarity_threshold']:
                self.candidate_selector.add_match(
                    EdgeInterRobot(self.params['robot_id'], kf, msg.robot_id,
                                   msg.image_id, similarity))

    def match_local_loop_closures(self, descriptor, kf_id):
        kfs, similarities = self.local_nnsm.search(descriptor,
                                         k=self.params['frontend.nb_best_matches'])

        if len(kfs) > 0 and kfs[0] == kf_id:
            kfs, similarities = kfs[1:], similarities[1:]
        if len(kfs) == 0:
            return None, None

        for kf, similarity in zip(kfs, similarities):
            if abs(kf -
                   kf_id) < self.params['frontend.intra_loop_min_inbetween_keyframes']:
                continue

            if similarity < self.params['frontend.similarity_threshold']:
                continue

            return kf, kfs
        return None, None

    def select_candidates(self,
                          number_of_candidates,
                          is_neighbor_in_range,
                          greedy_initialization=True):
        """Select inter-robot loop closure candidates according to budget

        Args:
            number_of_candidates (int): inter-robot loop closure budget,
            is_neighbor_in_range: dict(int, bool): indicates which other robots are in communication range 
            greedy_initialization: bool: use greedy initialization for selection

        Returns:
            list(EdgeInterRobot): selected edges
        """
        if len(self.candidate_selector.candidate_edges) > number_of_candidates:
            return self.candidate_selector.select_candidates(
                number_of_candidates, is_neighbor_in_range,
                greedy_initialization)
        else:
            return list(self.candidate_selector.candidate_edges.values())
