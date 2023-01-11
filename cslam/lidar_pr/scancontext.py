import cslam.lidar_pr.scancontext_utils as sc_utils

class ScanContext:
    """
    Scan Context descriptor for point clouds
    From: https://github.com/irapkaist/scancontext
    """
    def __init__(self, params, node):
        self.node = node
        self.params = params
        self.shape = [20,60] # Same as in ScanContext paper
        self.max_length = 80 # Same as in ScanContext paper

    def compute_embedding(self, keyframe):
        desc = sc_utils.ptcloud2sc(keyframe, self.shape, self.max_length)
        return desc.reshape(-1)
        