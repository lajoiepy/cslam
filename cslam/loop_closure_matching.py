from scipy.stats import logistic
import numpy as np
from cslam.nearest_neighbors_matching import NearestNeighborsMatching


class LoopClosureMatching(object):

    def __init__(self, params):
        self.params = params

        self.robot_id = self.params['robot_id']
        self.local_nnsm = NearestNeighborsMatching()
        self.other_robots_nnsm = {}
        self.local_keyframe_id = []
        self.other_robots_keyframes = {}
        self.other_robots_keyframe_similarities = {}
        self.nb_robots = self.params['nb_robots']
        for i in range(self.nb_robots):
            if i != self.robot_id:
                self.other_robots_nnsm[i] = NearestNeighborsMatching()
                self.other_robots_keyframes[i] = []
                self.other_robots_keyframe_similarities[i] = []
        self.similarity_loc = self.params['similarity_loc']
        self.similarity_scale = self.params['similarity_scale']

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
        self.local_nnsm.add_item(embedding, id)
        self.local_keyframe_id.append(id)
        for i in range(self.nb_robots):
            if i != self.robot_id:
                kf, d = self.other_robots_nnsm[i].search_best(embedding, k=1)
                if d <= self.params['threshold']:
                    self.other_robots_keyframes[i].append(kf)
                    self.other_robots_keyframe_similarities[i].append(
                        self.distance_to_similarity(d))
                else:
                    self.other_robots_keyframes[i].append(-1)
                    self.other_robots_keyframe_similarities[i].append(-1)

    def add_other_robot_keyframe(self, msg):
        self.other_robots_nnsm[msg.robot_id].add_item(
            np.asarray(msg.descriptor), msg.image_id)

        kf, d = self.local_nnsm.search_best(np.asarray(msg.descriptor))
        similarity = self.distance_to_similarity(d)
        if d <= self.params['threshold'] and similarity > self.best_matches[
                'robot_' + str(msg.robot_id) + '_similarity'][kf]:
            self.other_robots_keyframes[msg.robot_id][kf] = msg.image_id
            self.other_robots_keyframe_similarities[
                msg.robot_id][kf] = similarity
