#!/usr/bin/env python
from cslam.algebraic_connectivity_maximization import EdgeInterRobot
import numpy as np
from networkx.algorithms import bipartite
import networkx as nx


class Broker(object):
    """The broker decides which vertices in the matching 
    graph are to be shared between the robots.
    """

    def __init__(self, edges, robots_involved):
        """Initialize the broker

        Args:
            edges (list(EdgeInterRobot)): _description_
            robots_involved (list(int)): Robot ids of the 
                                robots involved in the exchange
        """
        self.edges = edges
        robots_involved_with_edges = set()
        for e in self.edges:
            if e.robot0_id in robots_involved:
                robots_involved_with_edges.add(e.robot0_id)
            if e.robot1_id in robots_involved:
                robots_involved_with_edges.add(e.robot1_id)
        robots_involved_with_edges = list(robots_involved_with_edges)
        self.is_multi_robot_graph = len(robots_involved_with_edges) >= 2

        if self.is_multi_robot_graph:
            if len(robots_involved_with_edges) == 2:
                self.is_bipartite = True
            elif len(robots_involved_with_edges) > 2:
                self.is_bipartite = False
            else:
                raise Exception(
                    "Broker: The communication brokerage needs to be between at least 2 robots"
                )

            # Build graph
            self.matching_graph = nx.Graph()
            for e in self.edges:
                edge_vertices = [(e.robot0_id, e.robot0_keyframe_id),
                                 (e.robot1_id, e.robot1_keyframe_id)]
                # Add vertices (required for bipartite)
                for vertex in edge_vertices:
                    if vertex not in self.matching_graph:
                        if self.is_bipartite:
                            if vertex[0] == robots_involved_with_edges[0]:
                                self.matching_graph.add_node(vertex,
                                                             bipartite=0)
                            elif vertex[0] == robots_involved_with_edges[1]:
                                self.matching_graph.add_node(vertex,
                                                             bipartite=1)
                        else:
                            if vertex[0] in robots_involved_with_edges:
                                self.matching_graph.add_node(vertex)
                # Add edges
                if edge_vertices[0][
                        0] in robots_involved_with_edges and edge_vertices[1][
                            0] in robots_involved_with_edges:
                    self.matching_graph.add_edge(edge_vertices[0],
                                                 edge_vertices[1])

    def brokerage(self, use_vertex_cover):
        """Return the broker selection of vertices to send.
        Either using vertex cover or simple dialog strategy.

        Args:
            use_vertex_cover (bool): use vertex cover stratehy

        Returns:
            List(set((int,int)): Vertices to be transmitted
        """
        if self.is_multi_robot_graph:
            if use_vertex_cover:
                return self.vertex_cover()
            else:
                return self.simple_dialog()
        else:
            return []

    def vertex_cover(self):
        """Computes the minimum vertex cover over the edges
        If the graph is bipartite: we compute the maximum matching and
         recover the minimum vertex cover with Konig's theorem

        Otherwise, we use an approximate greedy algorithm.

        Returns:
            List(set((int,int)): Vertices to be transmitted
        """
        vertex_covers = []
        # Divide into components
        matching_subgraphs = [
            self.matching_graph.subgraph(c).copy()
            for c in nx.connected_components(self.matching_graph)
        ]
        # Compute min cover on connected components
        for graph in matching_subgraphs:
            if self.is_bipartite:
                matching = nx.bipartite.maximum_matching(graph)
                vertex_covers.append(
                    nx.bipartite.to_vertex_cover(graph, matching))
            else:
                vertex_covers.append(
                    nx.algorithms.approximation.vertex_cover.
                    min_weighted_vertex_cover(graph))
        return vertex_covers

    def simple_dialog(self):
        """Simple dialog exchange
        For each edge, transmit one of the two vertices randomly
        unless one of the 2 vertices is already transmitted.

        Returns:
            List(set((int,int)): Vertices to be transmitted
        """
        vertices_dialog = set()
        for e in self.edges:
            edge_vertices = [(e.robot0_id, e.robot0_keyframe_id),
                             (e.robot1_id, e.robot1_keyframe_id)]
            if edge_vertices[0] not in vertices_dialog and edge_vertices[
                    1] not in vertices_dialog:
                rand_idx = np.random.randint(2)
                vertices_dialog.add(edge_vertices[rand_idx])

        return [vertices_dialog]
