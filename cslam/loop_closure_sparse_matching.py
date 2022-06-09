from scipy.stats import logistic
import numpy as np
from cslam.nearest_neighbors_matching import NearestNeighborsMatching
from cslam.algebraic_connectivity_maximization import AlgebraicConnectivityMaximization, EdgeInterRobot


class LoopClosureSparseMatching(object):
    """Sparse matching for loop closure detection
        Matches global descriptors to generate loop closure candidates
        Then candidates are selected such that we respect the communication budget
    """

    def __init__(self, params):
        """TODO

        Args:
            params (_type_): _description_
        """
        # Extract params
        self.params = params
        self.robot_id = self.params['robot_id']
        self.nb_robots = self.params['nb_robots']
        self.threshold = self.params['threshold']
        self.similarity_loc = self.params['similarity_loc']
        self.similarity_scale = self.params['similarity_scale']
        # Initialize matching structs
        self.local_nnsm = NearestNeighborsMatching()
        self.other_robots_nnsm = {}
        for i in range(self.nb_robots):
            if i != self.robot_id:
                self.other_robots_nnsm[i] = NearestNeighborsMatching()
        # Initialize candidate selection algorithm
        self.candidate_selector = AlgebraicConnectivityMaximization(self.robot_id, self.nb_robots)

    def distance_to_similarity(self, distance):
        """Converts a distance metric into a similarity score

        Args:
            distance (float): Place recognition distance metric

        Returns:
            float: similarity score
        """
        return logistic.cdf(-distance,
                            loc=self.similarity_loc,
                            scale=self.similarity_scale)

    def add_local_keyframe(self, embedding, id):
        """TODO

        Args:
            embedding (_type_): _description_
            id (_type_): _description_
        """
        self.local_nnsm.add_item(embedding, id)
        for i in range(self.nb_robots):
            if i != self.robot_id:
                kf, d = self.other_robots_nnsm[i].search_best(embedding, k=1)
                similarity = self.distance_to_similarity(d)
                if similarity >= self.threshold:
                    self.candidate_selector.add_match(EdgeInterRobot(self.robot_id, id, i, kf))

    def add_other_robot_keyframe(self, msg):
        """TODO

        Args:
            msg (_type_): _description_
        """
        self.other_robots_nnsm[msg.robot_id].add_item(
            np.asarray(msg.descriptor), msg.image_id)

        kf, d = self.local_nnsm.search_best(np.asarray(msg.descriptor))
        similarity = self.distance_to_similarity(d)
        if similarity >= self.threshold:
            self.candidate_selector.add_match(EdgeInterRobot(self.robot_id, kf, msg.robot_id, msg.image_id))

    def select_candidates(self, number_of_candidates):
        """TODO

        Args:
            number_of_candidates (_type_): _description_
        """
        return self.candidate_selector.select_candidates(number_of_candidates)
