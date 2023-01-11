import cslam.lidar_pr.scancontext_utils as sc_utils
import numpy as np
from scipy import spatial

class ScanContextMatching(object):
    """Nearest Neighbor matching of description vectors
    """

    def __init__(self, shape=[20,60], num_candidates=10, threshold=0.15): 
        """ Initialization
            Default configs are the same as in the original paper 

        """
        self.shape = shape
        self.num_candidates = num_candidates
        self.threshold = threshold

        self.scancontexts = np.zeros((1000, self.shape[0], self.shape[1]))
        self.ringkeys = np.zeros((1000, self.shape[0]))
        self.items = dict()
        self.nb_items = 0

    def add_item(self, descriptor, item):
        """Add item to the matching list

        Args:
            descriptor (np.array): descriptor
            item: identification info (e.g., int)
        """
        sc = descriptor.reshape(self.shape)

        if self.nb_items >= len(self.ringkeys):
            self.scancontexts.resize((2 * len(self.scancontexts), self.shape[0], self.shape[1]),
                                 refcheck=False)
            self.ringkeys.resize((2 * len(self.ringkeys), self.shape[0]),
                                 refcheck=False)
                                 
        rk = sc_utils.sc2rk(sc)

        self.scancontexts[self.nb_items] = sc
        self.ringkeys[self.nb_items] = rk
        self.items[self.nb_items] = item

        self.nb_items = self.nb_items + 1

    def search(self, query, k):
        """Search for nearest neighbors

        Args:
            query (np.array): descriptor to match
            k (int): number of best matches to return

        Returns:
            list(int, np.array): best matches
        """
        if self.nb_items < 1:
            return [None], [None]

        # step 1
        ringkey_history = np.array(self.ringkeys[:self.nb_items])
        ringkey_tree = spatial.KDTree(ringkey_history)

        query_sc = query.reshape(self.shape)
        ringkey_query = sc_utils.sc2rk(query_sc)
        _, nncandidates_idx = ringkey_tree.query(ringkey_query, k=self.num_candidates)

        # step 2        
        nn_dist = 1.0 # initialize with the largest value of distance
        nn_idx = None
        nn_yawdiff = None
        for ith in range(self.num_candidates):
            candidate_idx = nncandidates_idx[ith]
            candidate_sc = self.scancontexts[candidate_idx]
            dist, yaw_diff = sc_utils.distance_sc(candidate_sc, query_sc)
            if(dist < nn_dist):
                nn_dist = dist
                nn_yawdiff = yaw_diff
                nn_idx = candidate_idx

        if nn_idx is None:
            nn_idx = 0 
            nn_yawdiff_deg = 0
            similarity = 0.0
        else:
            nn_yawdiff_deg = nn_yawdiff * (360/self.shape[1])
            similarity = 1 - nn_dist # For now we return only 1 match, but we could return the n best matches
        return [self.items[nn_idx]], [similarity]

    def search_best(self, query):
        """Search for the nearest neighbor
            Implementation for compatibily only

        Args:
            query (np.array): descriptor to match

        Returns:
            int, np.array: best match
        """
        if self.nb_items < 1:
            return None, None
            
        idxs, sims = self.search(query, 1)

        return idxs[0], sims[0]
