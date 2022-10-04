
class ScanContextMatching(object):
    """Nearest Neighbor matching of description vectors
    """

    def __init__(self):
        """Initialization

        """
        pass

    def add_item(self, vector, item):
        """Add item to the matching list

        Args:
            vector (np.array): descriptor
            item: identification info (e.g., int)
        """
        pass

    def search(self, query, k):  # searching from 100000 items consume 30ms
        """Search for nearest neighbors

        Args:
            query (np.array): descriptor to match
            k (int): number of best matches to return

        Returns:
            list(int, np.array): best matches
        """
        pass

    def search_best(self, query):
        """Search for the nearest neighbor

        Args:
            query (np.array): descriptor to match

        Returns:
            int, np.array: best match
        """
        pass