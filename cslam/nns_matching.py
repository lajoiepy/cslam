#!/usr/bin/env python
import numpy as np
from scipy.spatial import distance


class NearestNeighborsMatching(object):
    """Nearest Neighbor matching of description vectors
    """

    def __init__(self, dim=None):
        """Initialization

        Args:
            dim (int, optional): Global descriptor size. Defaults to None.
        """
        self.n = 0
        self.dim = dim
        self.items = dict()
        self.data = []
        if dim is not None:
            self.data = np.zeros((1000, dim), dtype='float32')

    def add_item(self, vector, item):
        """Add item to the matching list

        Args:
            vector (np.array): descriptor
            item: identification info (e.g., int)
        """
        assert vector.ndim == 1
        if self.n >= len(self.data):
            if self.dim is None:
                self.dim = len(vector)
                self.data = np.zeros((1000, self.dim), dtype='float32')
            else:
                self.data.resize((2 * len(self.data), self.dim),
                                 refcheck=False)
        self.items[self.n] = item
        self.data[self.n] = vector
        self.n += 1

    def search(self, query, k):  # searching from 100000 items consume 30ms
        """Search for nearest neighbors

        Args:
            query (np.array): descriptor to match
            k (int): number of best matches to return

        Returns:
            list(int, np.array): best matches
        """
        if len(self.data) == 0:
            return [], []

        similarities = np.zeros(self.n)

        for i in range(self.n):
            similarities[i] = 1 - distance.cosine(query, self.data[i,:].squeeze())

        ns = np.argsort(similarities)[::-1][:k]
        return [self.items[n] for n in ns], similarities[ns]

    def search_best(self, query):
        """Search for the nearest neighbor

        Args:
            query (np.array): descriptor to match

        Returns:
            int, np.array: best match
        """
        if len(self.data) == 0:
            return None, None

        items, similarities = self.search(query, 1)
        return items[0], similarities[0]