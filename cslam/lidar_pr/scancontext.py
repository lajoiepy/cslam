import cslam.lidar_pr.scancontext_utils as sc_utils

# TODO: document
class ScanContext:
    def __init__(self, params, node):
        self.node = node
        self.params = params
        self.shape = [20,60] # Same as in ScanContext paper
        self.max_length = 80 # Same as in ScanContext paper

    def compute_embedding(self, keyframe):
        return sc_utils.ptcloud2sc(keyframe.points, self.shape, self.max_length)
        