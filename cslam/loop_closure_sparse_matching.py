import numpy as np
from cslam.nearest_neighbors_matching import NearestNeighborsMatching
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
        self.robot_id = self.params['robot_id']
        self.nb_robots = self.params['nb_robots']
        self.threshold = self.params['similarity_threshold']
        self.similarity_loc = self.params['similarity_loc']
        self.similarity_scale = self.params['similarity_scale']
        # Initialize matching structs
        self.local_nnsm = NearestNeighborsMatching()
        self.other_robots_nnsm = {}
        for i in range(self.nb_robots):
            if i != self.robot_id:
                self.other_robots_nnsm[i] = NearestNeighborsMatching()
        # Initialize candidate selection algorithm
        self.candidate_selector = AlgebraicConnectivityMaximization(
            self.robot_id, self.nb_robots)

    def distance_to_similarity(self, distance):
        """Converts a distance metric into a similarity score

        Args:
            distance (float): Place recognition distance metric

        Returns:
            float: similarity score
        """
        return 1 / (1 + np.exp(
            (distance - self.similarity_loc) / self.similarity_scale))

    def add_local_keyframe(self, embedding, id):
        """ Add a local keyframe for matching

        Args:
            embedding (np.array): global descriptor
            id (int): keyframe id
        """
        self.local_nnsm.add_item(embedding, id)
        for i in range(self.nb_robots):
            if i != self.robot_id:
                kf, d = self.other_robots_nnsm[i].search_best(embedding)
                if kf is not None:
                    similarity = self.distance_to_similarity(d)
                    if similarity >= self.threshold:
                        self.candidate_selector.add_match(
                            EdgeInterRobot(self.robot_id, id, i, kf,
                                           similarity))

    def add_other_robot_keyframe(self, msg):
        """ Add keyframe global descriptor info from other robot

        Args:
            msg (cslam_loop_detection_interfaces.msg.GlobalImageDescriptor): global descriptor info
        """
        self.other_robots_nnsm[msg.robot_id].add_item(
            np.asarray(msg.descriptor), msg.image_id)

        kf, d = self.local_nnsm.search_best(np.asarray(msg.descriptor))
        if kf is not None:
            similarity = self.distance_to_similarity(d)
            if similarity >= self.threshold:
                self.candidate_selector.add_match(
                    EdgeInterRobot(self.robot_id, kf, msg.robot_id,
                                   msg.image_id, similarity))

    def match_local_loop_closures(self, descriptor):
        kfs, ds = self.local_nnsm.search(descriptor,
                                         k=self.params['nb_best_matches'])

        if len(kfs) > 0 and kfs[0] == id:
            kfs, ds = kfs[1:], ds[1:]
        if len(kfs) == 0:
            return None

        for kf, d in zip(kfs, ds):
            if abs(kf - id) < self.params['intra_loop_min_inbetween_keyframes']:
                continue

            if d > self.params['similarity_threshold']:
                continue

            return kf, kfs
        return None, None

    def select_candidates(self, number_of_candidates, is_neighbor_in_range):
        """Select inter-robot loop closure candidates according to budget

        Args:
            number_of_candidates (int): inter-robot loop closure budget
            is_neighbor_in_range: dict(int, bool): indicates which other robots are in communication range 

        Returns:
            list(EdgeInterRobot): selected edges
        """
        if len(self.candidate_selector.candidate_edges
               ) >= number_of_candidates:
            return self.candidate_selector.select_candidates(
                number_of_candidates, is_neighbor_in_range)
        else:
            return []
