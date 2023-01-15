from typing import NamedTuple

import numpy as np

from cslam.mac.mac import MAC
from cslam.mac.utils import Edge, weight_graph_lap_from_edge_list


class EdgeInterRobot(NamedTuple):
    """ Inter-robot loop closure edge
    """
    robot0_id: int
    robot0_keyframe_id: int
    robot1_id: int
    robot1_keyframe_id: int
    weight: float

    def __eq__(self, other):
        """ Overload the equal operator in order to ignore the weights

        Args:
            other (EdgeInterRobot): Other edge to compare
        """
        return ((self.robot0_id == other.robot0_id) and
                (self.robot0_keyframe_id == other.robot0_keyframe_id) and
                (self.robot1_id == other.robot1_id) and
                (self.robot1_keyframe_id == other.robot1_keyframe_id)) or (
                    (self.robot0_id == other.robot1_id) and
                    (self.robot0_keyframe_id == other.robot1_keyframe_id) and
                    (self.robot1_id == other.robot0_id) and
                    (self.robot1_keyframe_id == other.robot0_keyframe_id))


class AlgebraicConnectivityMaximization(object):

    def __init__(
        self,
        robot_id=0,
        max_nb_robots=1,
        max_iters=20,
        fixed_weight=1.0,
        extra_params={
            "frontend.enable_sparsification": True,
            "evaluation.enable_sparsification_comparison": False,
        }):
        """Initialization

        Args:
            robot_id (int, optional): ID of the robot
            max_nb_robots (int, optional): number of robots. Defaults to 1.
            max_iters (int, optional): maximum number of iterations. Defaults to 20.
            fixed_weight (float, optional): weight of fixed measurements. Defaults to 1.0.
        """
        self.fixed_weight = fixed_weight
        self.params = extra_params

        self.fixed_edges = []
        self.candidate_edges = {}
        self.already_considered_matches = set()

        self.max_iters = max_iters

        self.max_nb_robots = max_nb_robots
        self.robot_id = robot_id
        self.total_nb_poses = 0

        self.initial_fixed_edge_exists = {}
        self.nb_poses = {}
        for i in range(self.max_nb_robots):
            self.nb_poses[i] = 0
            self.initial_fixed_edge_exists[i] = False

        self.log_greedy_edges = []
        self.log_mac_edges = []

    def edge_key(self, edge):
        """Get a unique key for an edge

        Args:
            edge (EdgeInterRobot): inter-robot edge

        Returns:
            tuple(int, int, int, int): unique key
        """
        if edge.robot0_id < edge.robot1_id:
            return (edge.robot0_id, edge.robot0_keyframe_id, edge.robot1_id,
                    edge.robot1_keyframe_id)
        else:
            return (edge.robot1_id, edge.robot1_keyframe_id, edge.robot0_id,
                    edge.robot0_keyframe_id)

    def replace_weight(self, edge, weight):
        """Replace the weight of an edge

        Args:
            edge (EdgeInterRobot): inter-robot edge
            weight (float): new weight

        Returns:
            EdgeInterRobot: new edge with new weight
        """
        # Check if inter-robot edge
        if type(edge) is EdgeInterRobot:
            return EdgeInterRobot(edge.robot0_id, edge.robot0_keyframe_id,
                                  edge.robot1_id, edge.robot1_keyframe_id,
                                  weight)
        elif type(edge) is Edge:
            return Edge(edge.i, edge.j, weight)

    def update_nb_poses(self, edge):
        """The number of poses should be the maximal edge id known

        Args:
            edge (EdgeInterRobot): loop closure edge
        """
        self.nb_poses[edge.robot0_id] = max(self.nb_poses[edge.robot0_id],
                                            edge.robot0_keyframe_id + 1)
        self.nb_poses[edge.robot1_id] = max(self.nb_poses[edge.robot1_id],
                                            edge.robot1_keyframe_id + 1)

    def update_initial_fixed_edge_exists(self, fixed_edge):
        """Maintains a bool for each neighbors to know if we have at least a known link.
        In cases where no initial fixed edge exists, we perform the greedy selection

        Args:
            fixed_edge (EdgeInterRobot): fixed edge in the graph
        """
        if fixed_edge.robot0_id != fixed_edge.robot1_id:
            self.initial_fixed_edge_exists[fixed_edge.robot0_id] = True
            self.initial_fixed_edge_exists[fixed_edge.robot1_id] = True

    def set_graph(self, fixed_edges, candidate_edges):
        """Fill graph struct

        Args:
            fixed_edges (list(EdgeInterRobot)): edges that are already computed
            candidate_edges (list(EdgeInterRobot)): candidate edges to compute
        """
        self.fixed_edges = fixed_edges

        # Extract nb of poses for ids graphs
        for e in self.fixed_edges:
            self.update_nb_poses(e)
            self.update_initial_fixed_edge_exists(e)

        for e in candidate_edges:
            self.update_nb_poses(e)

        for e in candidate_edges:
            self.candidate_edges[self.edge_key(e)] = e

    def add_fixed_edge(self, edge):
        """Add an already computed edge to the graph

        Args:
            edge (EdgeInterRobot): inter-robot edge
        """
        self.fixed_edges.append(edge)
        # Update nb of poses and initial edge check
        self.update_nb_poses(edge)
        self.update_initial_fixed_edge_exists(edge)

    def add_candidate_edge(self, edge):
        """Add a candidate edge to the graph

        Args:
            edge (EdgeInterRobot): inter-robot edge
        """
        # Check if the edge is not a failed edge or fixed edge
        if self.edge_key(edge) in self.already_considered_matches:
            return

        # Otherwise add it to the candidate edges
        self.candidate_edges[self.edge_key(edge)] = edge
        # Update nb of poses
        self.update_nb_poses(edge)

    def remove_candidate_edges(self, edges, failed=False):
        """Remove candidate edge from the graph

        Args:
            edges (list(EdgeInterRobot)): inter-robot edges
        """
        keys = list(self.candidate_edges.keys())
        for k in keys:
            if self.candidate_edges[k] in edges:
                del self.candidate_edges[k]

        for edge in edges:
            self.already_considered_matches.add(self.edge_key(edge))

    def candidate_edges_to_fixed(self, edges):
        """Move candidate edges to fixed. 
        Use when candidates are successfully converted into measurements

        Args:
            edges (list(EdgeInterRobot)): inter-robot edges
        """
        for i in range(len(edges)):
            edges[i] = self.replace_weight(edges[i], weight=self.fixed_weight)
            self.update_initial_fixed_edge_exists(edges[i])
        self.fixed_edges.extend(edges)
        self.remove_candidate_edges(edges)

    def greedy_initialization(self, nb_candidates_to_choose, edges):
        """Greedy weight initialization

        Args:
            nb_candidates_to_choose (int): number of edges to choose
            edges (list(Edge)): candidate_edges
        """
        weights = [e.weight for e in edges]
        nb_edges = len(weights)
        w_init = np.zeros(nb_edges)
        indices = np.argpartition(
            weights, -nb_candidates_to_choose)[-nb_candidates_to_choose:]
        w_init[indices] = 1.0
        return w_init

    def pseudo_greedy_initialization(self, nb_candidates_to_choose, nb_random,
                                     edges):
        """Greedy weight initialization
            Greedy initialization with for the first nb_candidates_to_choose-nb_random
            then random choice

        Args:
            nb_candidates_to_choose (int): number of edges to choose
            nb_random (int): number of edges to choose randomly
            edges (list(Edge)): candidate_edges
        """
        nb_greedy = nb_candidates_to_choose - nb_random
        w_init = self.greedy_initialization(nb_greedy, edges)
        nb_edges = len(edges)
        i = 0
        trial = 0
        max_trials = 2 * nb_random
        while i < nb_random and trial < max_trials:
            j = int(np.random.rand() * nb_edges)
            if w_init[j] < 0.5:
                w_init[j] = 1.0
                i = i + 1
            trial += 1
        if trial >= max_trials:
            w_init = self.greedy_initialization(nb_candidates_to_choose, edges)
        return w_init

    def random_initialization(self, nb_candidates_to_choose, edges):
        """Random weight initialization

        Args:
            nb_candidates_to_choose (int): number of edges to choose
        """
        for e in range(len(edges)):
            edges[e] = self.replace_weight(edges[e], np.random.rand())
        return self.greedy_initialization(nb_candidates_to_choose, edges)

    def connection_biased_greedy_selection(self, nb_candidates_to_choose,
                                           edges, is_robot_included):
        """Greedy weight initialization with connection bias
        Prioritize edges that connect unconnected robots.
        """
        nb_candidate_chosen = 0
        edges_copy = edges.copy()
        edges_ids_to_select = []
        rids = [r for r in is_robot_included.keys() if is_robot_included[r]]
        for rid in rids:
            if not self.initial_fixed_edge_exists[rid]:
                max_weight = -1
                max_edge = None
                for i in range(len(edges_copy)):
                    if edges_copy[i].robot0_id == rid or edges_copy[
                            i].robot1_id == rid:
                        if edges_copy[i].weight > max_weight:
                            max_weight = edges_copy[i].weight
                            max_edge = i
                if max_edge is not None:
                    edges_ids_to_select.append(max_edge)
                    edges_copy[max_edge] = self.replace_weight(
                        edges_copy[max_edge], weight=0.0)
                    nb_candidate_chosen += 1

        w_init = np.zeros(len(edges))
        if nb_candidates_to_choose - nb_candidate_chosen > 0:
            w_init = self.greedy_initialization(
                nb_candidates_to_choose - nb_candidate_chosen,
                self.rekey_edges(edges_copy, is_robot_included))
        for i in edges_ids_to_select:
            w_init[i] = 1.0
        return w_init

    def compute_offsets(self, is_robot_included):
        """Compute rekey offsets

        Args:
            is_robot_included dict(int, bool): Indicates if the robot 
                                is connected and in communication range
        """
        # Offsets required to put rekey nodes such
        # that they are all in a single graph
        self.offsets = {}
        for i in range(self.max_nb_robots):
            self.offsets[i] = 0
        # Compute offsets
        previous_offset = 0
        previous_nb_poses = 0
        for id in range(self.max_nb_robots):
            if is_robot_included[id]:
                self.offsets[id] = previous_offset + previous_nb_poses
                previous_offset = self.offsets[id]
                previous_nb_poses = self.nb_poses[id]

    def rekey_edges(self, edges, is_robot_included):
        """Modify keys (nodes ID) from robot_id+keyframe_id to node_id
        Result example: 3 robots with 10 nodes eachs
        robot 0 nodes id = 1 to 9
        robot 1 nodes id = 10 to 19
        robot 2 nodes id = 20 to 29

        Args:
            edges (dict(EdgeInterRobot)): inter-robot edges
            is_robot_included dict(int, bool): Indicates if the robot 
                                is connected and in communication range

        Returns:
            list(Edge): edges with keys for MAC problem
        """
        # Rekey edges
        rekeyed_edges = []
        for e in edges:
            if is_robot_included[e.robot0_id] and is_robot_included[
                    e.robot1_id]:
                i = self.offsets[e.robot0_id] + e.robot0_keyframe_id
                j = self.offsets[e.robot1_id] + e.robot1_keyframe_id
                rekeyed_edges.append(Edge(i, j, e.weight))
        return rekeyed_edges

    def get_included_edges(self, edges, is_robot_included):
        """TODO
        """
        # Rekey edges
        included_edges = []
        for e in edges:
            if is_robot_included[e.robot0_id] and is_robot_included[
                    e.robot1_id]:
                included_edges.append(e)
        return included_edges

    def fill_odometry(self):
        """Add odometry edges
        We can infer the odometry edges directly from the number of poses,
        without communication.

        Returns:
            list(Edge): odometry edges
        """
        odom_edges = []
        for i in range(len(self.nb_poses)):
            for k in range(self.nb_poses[i] - 1):
                odom_edges.append(
                    Edge(self.offsets[i] + k, self.offsets[i] + k + 1,
                         self.fixed_weight))
        return odom_edges

    def recover_inter_robot_edges(self, edges, is_robot_included):
        """Recover IDs from before rekey_edges()

        Args:
            edges (list(Edge)): rekeyed edges
            is_robot_included (dict(int, bool)): indicates if a robot is included

        Returns:
            list(EdgeInterRobot): edges
        """
        recovered_inter_robot_edges = []
        for c in range(len(edges)):
            robot0_id = 0
            robot1_id = 0
            for o in self.offsets:
                if o != 0:
                    if is_robot_included[o] and edges[c].i >= self.offsets[o]:
                        robot0_id = o
                    if is_robot_included[o] and edges[c].j >= self.offsets[o]:
                        robot1_id = o
            robot0_keyframe_id = edges[c].i - self.offsets[robot0_id]
            robot1_keyframe_id = edges[c].j - self.offsets[robot1_id]
            recovered_inter_robot_edges.append(
                EdgeInterRobot(robot0_id, robot0_keyframe_id, robot1_id,
                               robot1_keyframe_id, edges[c].weight))
        return recovered_inter_robot_edges

    def check_graph_disconnections(self, is_other_robot_considered):
        """Check if the current graph of potential matches is connected

        Args:
            is_other_robot_considered: dict(int, bool): indicates which 
                            other robots are are to be included 
        
        Returns:
            dict(int, bool): dict indicating if each robot is connected
        """
        is_robot_connected = {}
        for i in range(self.max_nb_robots):
            if i == self.robot_id:
                is_robot_connected[i] = True
            else:
                is_robot_connected[i] = False
        for edge in self.fixed_edges:
            if is_other_robot_considered[edge.robot0_id]:
                is_robot_connected[edge.robot0_id] = True
            if is_other_robot_considered[edge.robot1_id]:
                is_robot_connected[edge.robot1_id] = True
        for edge in self.candidate_edges.values():
            if is_other_robot_considered[edge.robot0_id]:
                is_robot_connected[edge.robot0_id] = True
            if is_other_robot_considered[edge.robot1_id]:
                is_robot_connected[edge.robot1_id] = True
        return is_robot_connected

    def check_initial_fixed_measurements_exists(self, is_robot_included):
        """Check if we have an initial fixed measurement with each robot included
        If not, greedy selection should be used

        Args:
            is_robot_included dict(bool): indicates if each robot is included

        Returns:
            bool: check result
        """
        initial_fixed_measurements_exists = True
        for rid in is_robot_included:
            if is_robot_included[rid] and (
                    not self.initial_fixed_edge_exists[rid]):
                initial_fixed_measurements_exists = False
        return initial_fixed_measurements_exists

    def run_mac_solver(self, fixed_edges, candidate_edges, w_init,
                       nb_candidates_to_choose):
        """Run the maximalization of algebraic connectivity

        Args:
            fixed_edges list(EdgeInterRobot): fixed edges
            candidate_edges list(EdgeInterRobot): candidate edges
            w_init (np.array): One-hot selection vector
            nb_candidates_to_choose (int): budger
        """
        mac = MAC(fixed_edges, candidate_edges, self.total_nb_poses)

        result = w_init.copy()
        trial = 0
        while trial < nb_candidates_to_choose:
            try:
                result, _, _ = mac.fw_subset(w_init,
                                             nb_candidates_to_choose,
                                             max_iters=self.max_iters)
                break
            except:
                # This should happend very rarely.
                # find_fieldler_pair triggers a singular matrix exception
                # when the mac select measurements leading to graph disconnection.
                # Once at least one measurement is fixed connecting each robot it won't happen.
                # We vary with increasing randomness the initial guess until we reach a viable solution.
                trial = trial + 1
                w_init = self.pseudo_greedy_initialization(
                    nb_candidates_to_choose, trial, candidate_edges)
                continue
        return result

    def select_candidates(self,
                          nb_candidates_to_choose,
                          is_other_robot_considered,
                          greedy_initialization=True):
        """Solve algebraic connectivity maximization

        Args:
            nb_candidates_to_choose (int): number of candidates to choose,
                            related to a computation/communication budget
            is_other_robot_considered: dict(int, bool): indicates which 
                            other robots are in communication range 
            greedy_initialization: perform greedy initialization based on similarity

        Returns:
            list(EdgeInterRobot): selected edges
        """
        # Check connectivity
        is_robot_included = self.check_graph_disconnections(
            is_other_robot_considered)

        # Rekey multi-robot edges to single graph
        self.compute_offsets(is_robot_included)
        rekeyed_fixed_edges = self.rekey_edges(self.fixed_edges,
                                               is_robot_included)
        rekeyed_fixed_edges.extend(self.fill_odometry())
        rekeyed_candidate_edges = self.rekey_edges(
            self.candidate_edges.values(), is_robot_included)

        if nb_candidates_to_choose > len(rekeyed_candidate_edges):
            nb_candidates_to_choose = len(rekeyed_candidate_edges)

        if len(rekeyed_candidate_edges) > 0:
            # Compute number of poses
            self.total_nb_poses = 0
            for n in range(len(self.nb_poses)):
                self.total_nb_poses = self.total_nb_poses + self.nb_poses[n]

            # Initial guess
            if greedy_initialization:
                w_init = self.greedy_initialization(nb_candidates_to_choose,
                                                    rekeyed_candidate_edges)
            else:
                w_init = self.random_initialization(nb_candidates_to_choose,
                                                    rekeyed_candidate_edges)

            if self.params[
                    "frontend.enable_sparsification"] and self.check_initial_fixed_measurements_exists(
                        is_robot_included):
                result = self.run_mac_solver(rekeyed_fixed_edges,
                                             rekeyed_candidate_edges, w_init,
                                             nb_candidates_to_choose)
            else:
                result = self.connection_biased_greedy_selection(
                    nb_candidates_to_choose,
                    self.get_included_edges(self.candidate_edges.values(),
                                            is_robot_included),
                    is_robot_included)

            if self.params["evaluation.enable_sparsification_comparison"]:
                self.sparsification_comparison_logs(rekeyed_candidate_edges,
                                                    is_robot_included, w_init,
                                                    result)

            selected_edges = [
                rekeyed_candidate_edges[i]
                for i in np.nonzero(result.astype(int))[0]
            ]

            # Return selected multi-robot edges
            inter_robot_edges = self.recover_inter_robot_edges(
                selected_edges, is_robot_included)
            self.remove_candidate_edges(inter_robot_edges)

            return inter_robot_edges
        else:
            return []

    def sparsification_comparison_logs(self, rekeyed_candidate_edges,
                                       is_robot_included, greedy_result,
                                       mac_result):
        """ TODO: document
        """
        self.log_greedy_edges = self.recover_inter_robot_edges([
            rekeyed_candidate_edges[i]
            for i in np.nonzero(greedy_result.astype(int))[0]
        ], is_robot_included)
        self.log_mac_edges = self.recover_inter_robot_edges([
            rekeyed_candidate_edges[i]
            for i in np.nonzero(mac_result.astype(int))[0]
        ], is_robot_included)

    def add_match(self, match):
        """Add match

        Args:
            match (EdgeInterRobot): potential match
        """
        key = (match.robot0_id, match.robot0_keyframe_id, match.robot1_id,
               match.robot1_keyframe_id)

        if key in self.candidate_edges:
            if match.weight > self.candidate_edges[key].weight:
                self.add_candidate_edge(match)
        else:
            self.add_candidate_edge(match)
