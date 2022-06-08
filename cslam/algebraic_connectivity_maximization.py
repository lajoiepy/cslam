from cslam.third_party.mac.mac import MAC
from cslam.third_party.mac.utils import Edge
import numpy as np
from collections import namedtuple

EdgeInterRobot = namedtuple(
    'EdgeInterRobot',
    ['robot0_id', 'robot0_image_id', 'robot1_id', 'robot1_image_id', 'weight'])


class AlgebraicConnectivityMaximization(object):

    def __init__(self, robot_id=0, nb_robots=1, max_iters=20, fixed_weight=1.0):
        """Initialization

        Args:
            robot_id (int, optional): ID of the robot
            nb_robots (int, optional): number of robots. Defaults to 1.
            max_iters (int, optional): maximum number of iterations. Defaults to 20.
            fixed_weight (float, optional): weight of fixed measurements. Defaults to 1.0.
        """
        self.fixed_weight = fixed_weight

        self.fixed_edges = []
        self.candidate_edges = []

        self.max_iters = max_iters

        self.nb_robots = nb_robots
        self.robot_id = robot_id
        self.total_nb_poses = 0

        # Offsets required to put rekey nodes such
        # that they are all in a single graph
        self.offsets = {}
        self.nb_poses = {}
        for i in range(self.nb_robots):
            self.offsets[i] = 0
            self.nb_poses[i] = 0

    def set_graph(self, fixed_edges, candidate_edges):
        """Fill graph struct

        Args:
            fixed_edges (list(EdgeInterRobot)): edges that are already computed
            candidate_edges (list(EdgeInterRobot)): candidate edges to compute
        """
        self.fixed_edges = fixed_edges
        self.candidate_edges = candidate_edges

        # Extract nb of poses for ids graphs
        for e in self.fixed_edges:
            self.nb_poses[e.robot0_id] = max(self.nb_poses[e.robot0_id],
                                             e.robot0_image_id + 1)
            self.nb_poses[e.robot1_id] = max(self.nb_poses[e.robot1_id],
                                             e.robot1_image_id + 1)
        for e in self.candidate_edges:
            self.nb_poses[e.robot0_id] = max(self.nb_poses[e.robot0_id],
                                             e.robot0_image_id + 1)
            self.nb_poses[e.robot1_id] = max(self.nb_poses[e.robot1_id],
                                             e.robot1_image_id + 1)

    def add_fixed_edge(self, edge):
        """Add an already computed edge to the graph

        Args:
            edge (EdgeInterRobot): inter-robot edge
        """
        self.fixed_edges.append(edge)
        # Update nb of poses
        self.nb_poses[edge.robot0_id] = max(self.nb_poses[edge.robot0_id],
                                            edge.robot0_image_id + 1)
        self.nb_poses[edge.robot1_id] = max(self.nb_poses[edge.robot1_id],
                                            edge.robot1_image_id + 1)

    def add_candidate_edge(self, edge):
        """Add a candidate edge to the graph

        Args:
            edge (EdgeInterRobot): inter-robot edge
        """
        self.candidate_edges.append(edge)
        # Update nb of poses
        self.nb_poses[edge.robot0_id] = max(self.nb_poses[edge.robot0_id],
                                            edge.robot0_image_id + 1)
        self.nb_poses[edge.robot1_id] = max(self.nb_poses[edge.robot1_id],
                                            edge.robot1_image_id + 1)

    def remove_candidate_edges(self, edges):
        """Remove candidate edge from the graph

        Args:
            edge (EdgeInterRobot): inter-robot edge
        """
        self.candidate_edges = [
            self.candidate_edges[i] for i in range(len(self.candidate_edges))
            if (self.candidate_edges[i] not in edges)
        ]

    def candidate_edges_to_fixed(self, edges):
        """Move candidate edges to fixed. 
        Use when candidates are successfully converted into measurements

        Args:
            edges (list(EdgeInterRobot)): inter-robot edges
        """
        for i in range(len(edges)):
            edges[i]._replace(weight = self.fixed_weight)
        self.fixed_edges.extend(edges)
        self.remove_candidate_edges(edges)

    def greedy_intialization(self, weights, nb_candidates_to_choose):
        """Greedy weight initialization

        Args:
            weights (list(float)): weight for each candidate
            nb_candidates_to_choose (int): number of edges to choose
        """
        self.w_init = np.zeros(len(weights))
        indices = np.argpartition(
            weights, -nb_candidates_to_choose)[-nb_candidates_to_choose:]
        self.w_init[indices] = 1.0

    def random_initialization(self, nb_candidates_to_choose):
        """Random weight initialization

        Args:
            nb_candidates_to_choose (int): number of edges to choose
        """
        weights = np.random.rand(len(self.candidate_edges))
        self.greedy_intialization(weights, nb_candidates_to_choose)

    def rekey_edges(self, edges):
        """Modify keys (nodes ID) from robot_id+image_id to node_id
        Result example: 3 robots with 10 nodes eachs
        robot 0 nodes id = 1 to 9
        robot 1 nodes id = 10 to 19
        robot 2 nodes id = 20 to 29

        Args:
            edges (list(EdgeInterRobot)): inter-robot edges

        Returns:
            list(Edge): edges with keys for MAC problem
        """
        # Compute offsets
        for id in range(self.nb_robots)[1:]:
            self.offsets[id] = self.nb_poses[id - 1] + self.offsets[id - 1]
        # Rekey edges
        rekeyed_edges = []
        for e in edges:
            i = self.offsets[e.robot0_id] + e.robot0_image_id
            j = self.offsets[e.robot1_id] + e.robot1_image_id
            rekeyed_edges.append(Edge(i, j, e.weight))
        return rekeyed_edges

    def fill_other_robots_odometry(self):
        """Add odometry edges from other robots
        We can infer the odometry edges directly from the number of poses,
        without communication.

        Returns:
            list(Edge): odometry edges
        """
        odom_edges = []
        for i in range(len(self.nb_poses)):
            if i != self.robot_id:
                for k in range(self.nb_poses[i] - 1):
                    odom_edges.append(
                        Edge(self.offsets[i] + k, self.offsets[i] + k + 1,
                             self.fixed_weight))
        return odom_edges

    def recover_inter_robot_edges(self, edges):
        """Recover IDs from before rekey_edges()

        Args:
            edges (list(Edge)): rekeyed edges

        Returns:
            list(EdgeInterRobot): edges
        """
        recovered_inter_robot_edges = []
        for c in range(len(edges)):
            robot0_id = 0
            robot1_id = 0
            for o in range(len(self.offsets)):
                if o != 0:
                    if edges[c].i > self.offsets[o]:
                        robot0_id = robot0_id + 1
                    if edges[c].j > self.offsets[o]:
                        robot1_id = robot1_id + 1
            robot0_image_id = edges[c].i - self.offsets[robot0_id]
            robot1_image_id = edges[c].j - self.offsets[robot1_id]
            recovered_inter_robot_edges.append(
                EdgeInterRobot(robot0_id, robot0_image_id, robot1_id,
                               robot1_image_id, edges[c].weight))
        return recovered_inter_robot_edges

    def select_candidates(self, nb_candidates_to_choose, weights=None):
        """Solve algebraic connectivity maximization

        Args:
            nb_candidates_to_choose (int): number of candidates to choose,
                            related to a computation/communication budget
            weights (list(float), optional): weight of each candidate. Defaults to None.

        Returns:
            list(EdgeInterRobot): selected edges
        """
        # Rekey multi-robot edges to single robot
        rekeyed_fixed_edges = self.rekey_edges(self.fixed_edges)
        rekeyed_fixed_edges.extend(self.fill_other_robots_odometry())
        rekeyed_candidate_edges = self.rekey_edges(self.candidate_edges)

        # Compute number of poses
        self.total_nb_poses = 0
        for n in range(len(self.nb_poses)):
            self.total_nb_poses = self.total_nb_poses + self.nb_poses[n]

        # Initial guess
        if weights is None:
            self.random_initialization(nb_candidates_to_choose)
        else:
            self.greedy_intialization(weights, nb_candidates_to_choose)
        # Solver
        mac = MAC(rekeyed_fixed_edges, rekeyed_candidate_edges, self.total_nb_poses)
        result, _, _ = mac.fw_subset(self.w_init,
                                     nb_candidates_to_choose,
                                     max_iters=self.max_iters)
        selected_edges = [
            rekeyed_candidate_edges[i]
            for i in np.nonzero(result.astype(int))[0]
        ]
        # Return selected multi-robot edges
        return self.recover_inter_robot_edges(selected_edges)
