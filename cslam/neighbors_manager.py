from cslam.neighbor_monitor import NeighborMonitor
from cslam_common_interfaces.msg import RobotIdsAndOrigin
from std_msgs.msg import String

import rclpy


class NeighborManager():
    """Manage neighbors to keep track of which other robots are in communication range
    """
    def __init__(self, node, params): 
        self.node = node
        self.params = params
        self.robot_id = self.params['robot_id']
        self.max_nb_robots = self.params['max_nb_robots']
        self.neighbors_monitors = {}
        for rid in range(self.max_nb_robots):
            if rid != self.robot_id:
                self.neighbors_monitors[rid] = NeighborMonitor(
                    self.node, rid, self.
                    params['neighbor_management.enable_neighbor_monitoring'],
                    self.params['neighbor_management.init_delay_sec'],
                    self.params['neighbor_management.max_heartbeat_delay_sec'])

        self.subscriber = self.node.create_subscription(
            String, 'cslam/get_current_neighbors',
            self.get_current_neighbors_callback, 100)
        self.neighbors_publisher = self.node.create_publisher(
            RobotIdsAndOrigin, 'cslam/current_neighbors', 100)

    def check_neighbors_in_range(self):
        """Check which neighbors are in range
        
        """
        is_robot_in_range = {}
        robots_in_range_list = []
        for i in range(self.max_nb_robots):
            if i == self.robot_id:
                is_robot_in_range[i] = True
                robots_in_range_list.append(i)
            elif self.neighbors_monitors[i].is_alive():
                is_robot_in_range[i] = True
                robots_in_range_list.append(i)
            else:
                is_robot_in_range[i] = False
        return is_robot_in_range, robots_in_range_list

    def local_robot_is_broker(self):
        """This method check if the local robot (that runs this node), is the
        default broker based on its current neighbors.

        Returns:
            bool: is the local robot the default broker 
            among the robots in range
        """
        is_broker = True
        for i in range(self.max_nb_robots):
            if i != self.robot_id and self.neighbors_monitors[i].is_alive():
                # Note: This is an arbitrary condition that selects the
                # lowest ID alive as broker. Could be change to any other cond.
                # (e.g., selecting the robot with the most computing power)
                if self.robot_id > i:
                    is_broker = False
        return is_broker

    def select_from_which_kf_to_send(self, latest_local_id):
        """This function finds the range of descriptors to send
        so that we do not loose info
        """

        from_kf_id = latest_local_id
        for i in range(self.max_nb_robots):
            if i != self.robot_id:
                if self.neighbors_monitors[i].is_alive():
                    from_kf_id = min(
                        self.neighbors_monitors[i].last_keyframe_sent,
                        from_kf_id)

        for i in range(self.max_nb_robots):
            if i != self.robot_id:
                if self.neighbors_monitors[i].is_alive():
                    self.neighbors_monitors[
                        i].last_keyframe_sent = latest_local_id

        return from_kf_id + 1

    def select_from_which_match_to_send(self, latest_local_match_idx):
        """This function finds the range of matches to send
        so that we do not loose info
        """

        from_match_id = latest_local_match_idx
        for i in range(self.max_nb_robots):
            if i != self.robot_id:
                if self.neighbors_monitors[i].is_alive():
                    from_match_id = min(
                        self.neighbors_monitors[i].last_match_sent,
                        from_match_id)

        for i in range(self.max_nb_robots):
            if i != self.robot_id:
                if self.neighbors_monitors[i].is_alive():
                    self.neighbors_monitors[
                        i].last_match_sent = latest_local_match_idx

        return from_match_id + 1

    def useless_descriptors(self, last_kf_id):
        """Determines which descriptors are not useless and can be deleted

        Args:
            last_kf_id (int): last keyframe id in the list of descriptors
        Returns:
            int: the id of the first descriptor that is not useless
        """
        from_kf_id = last_kf_id
        for i in range(self.max_nb_robots):
            if i != self.robot_id:
                from_kf_id = min(self.neighbors_monitors[i].last_keyframe_sent,
                                 from_kf_id)
        return from_kf_id

    def useless_matches(self, last_match_id):
        """Determines which matches are not useless and can be deleted

        Args:
            last_match_id (int): last match id in the list of matches
        Returns:
            int: the id of the first match that is not useless
        """
        from_match_id = last_match_id
        for i in range(self.max_nb_robots):
            if i != self.robot_id:
                from_match_id = min(self.neighbors_monitors[i].last_match_sent,
                                    from_match_id)
        return from_match_id

    def update_received_kf_id(self, other_robot_id, kf_id):
        """Keep monitors up to date with received keyframes

        Args:
            other_robot_id (int): other robot id
            kf_id (int): keyframe id
        """
        self.neighbors_monitors[other_robot_id].last_keyframe_received = kf_id

    def get_unknown_range(self, descriptors):
        """_summary_

        Args:
            descriptors (list): list of descriptors with ids info

        Returns:
            range: indexes in list to process
        """
        other_robot_id = descriptors[0].robot_id
        kf_ids = [d.keyframe_id for d in descriptors]
        last_id = max(kf_ids)

        list_index_range = [
            i for i in range(len(descriptors)) if descriptors[i].keyframe_id >
            self.neighbors_monitors[other_robot_id].last_keyframe_received
        ]

        self.update_received_kf_id(
            other_robot_id,
            max(self.neighbors_monitors[other_robot_id].last_keyframe_received,
                last_id))
        return list_index_range

    def get_current_neighbors_callback(self, msg):
        """Publish the current neighbors in range

        Args:
            msg (String): Empty
        """
        is_robot_in_range, robots_in_range_list = self.check_neighbors_in_range(
        )
        robots_in_range_list.remove(self.robot_id)
        msg = RobotIdsAndOrigin()
        msg.robots.ids = robots_in_range_list
        for i in robots_in_range_list:
            msg.origins.ids.append(self.neighbors_monitors[i].origin_robot_id)

        self.neighbors_publisher.publish(msg)
