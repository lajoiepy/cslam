import cslam.lidar_pr.scancontext_utils as sc_utils

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

        self.max_length = 80 # recommended but other (e.g., 100m) is also ok.

        self.ENOUGH_LARGE = 15000 # capable of up to ENOUGH_LARGE number of nodes 
        self.scancontexts = [None] * self.ENOUGH_LARGE
        self.ringkeys = [None] * self.ENOUGH_LARGE

    def add_item(self, sc, item):
        """Add item to the matching list

        Args:
            sc (np.array): descriptor
            item: identification info (e.g., int)
        """
        rk = sc_utils.sc2rk(sc)

        self.scancontexts[node_idx] = sc
        self.ringkeys[node_idx] = rk

    def search(self, query, k):
        """Search for nearest neighbors

        Args:
            query (np.array): descriptor to match
            k (int): number of best matches to return

        Returns:
            list(int, np.array): best matches
        """
        if len(self.scancontexts) < 1:
            return [], []
        # step 1
        ringkey_history = np.array(self.ringkeys)
        ringkey_tree = spatial.KDTree(ringkey_history)

        ringkey_query = sc_utils.sc2rk(query)
        _, nncandidates_idx = ringkey_tree.query(ringkey_query, k=self.num_candidates)

        # step 2
        query_sc = query
        
        nn_dist = 1.0 # initialize with the largest value of distance
        nn_idx = None
        nn_yawdiff = None
        for ith in range(self.num_candidates):
            candidate_idx = nncandidates_idx[ith]
            candidate_sc = self.scancontexts[candidate_idx]
            dist, yaw_diff = distance_sc(candidate_sc, query_sc)
            if(dist < nn_dist):
                nn_dist = dist
                nn_yawdiff = yaw_diff
                nn_idx = candidate_idx

        if(nn_dist < self.threshold):
            nn_yawdiff_deg = nn_yawdiff * (360/self.shape[1])
            similarity = 1 - nn_dist # For now we return only 1 match
            return [nn_idx], [similarity]
        else:
            return [], []

    def search_best(self, query):
        """Search for the nearest neighbor
            Implementation for compatibily only

        Args:
            query (np.array): descriptor to match

        Returns:
            int, np.array: best match
        """
        if len(self.scancontexts) < 1:
            return None, None
            
        idxs, sims = self.search(query)
        return idxs[0], sims[0]