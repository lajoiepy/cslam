from cslam.neighbor_monitor import NeighborMonitor

class NeighborManager():
    def __init__(self, node, robot_id, nb_robots, max_delay_sec):
        self.node = node
        self.robot_id = robot_id
        self.nb_robots = nb_robots
        self.neighbors_monitors = {}
        for id in range(self.nb_robots):
            if id != self.robot_id:
                self.neighbors_monitors[id] = NeighborMonitor(self.node, id, max_delay_sec)

    def check_neighbors_in_range(self):
        """Check which neighbors are in range
        
        """
        is_robot_in_range = {}
        for i in range(self.nb_robots):
            if i == self.robot_id:
                is_robot_in_range[i] = True
            elif self.neighbors_monitors[i].is_alive():
                is_robot_in_range[i] = True
            else:
                is_robot_in_range[i] = False
        return is_robot_in_range

    def select_from_which_kf_to_send(self, latest_local_id):
        """This function finds the range of descriptors to send
        so that we do not loose info
        """
        from_kf_id = latest_local_id
        for i in range(self.nb_robots):
            if self.neighbors_monitors[i].is_alive():
                from_kf_id = min(self.neighbors_monitors[i].last_keyframe_sent, from_kf_id)
        
        for i in range(self.nb_robots):
            if self.neighbors_monitors[i].is_alive():
                self.neighbors_monitors[id].last_keyframe_sent = from_kf_id

        return from_kf_id

    def useless_descriptors(self, last_kf_id):
        """_summary_

        Args:
            last_index (int): last keyframe id in the list of descriptors
        """
        from_kf_id = last_kf_id
        for i in range(self.nb_robots):
            if i != self.robot_id:
                from_kf_id = min(self.neighbors_monitors[i].last_keyframe_sent, from_kf_id)
        return from_kf_id

    def update_received_kf_id(self, other_robot_id, kf_id):
        """Keep monitors up to date with received keyframes

        Args:
            other_robot_id (int): other robot id
            kf_id (int): keyframe id
        """
        self.neighbors_monitors[other_robot_id].last_keyframe_received = kf_id

    def get_unknown_range(self, start_id, end_id, other_robot_id):
        """_summary_

        Args:
            start_id (int): first keyframe id received
            end_id (int): last keyframe id received
            robot_id (int): other robot id

        Returns:
            range: indexes in list to process
        """
        if self.neighbors_monitors[other_robot_id].last_keyframe_received >= end_id:
            list_index_range = range()
        else:
            s = max(0, self.neighbors_monitors[other_robot_id].last_keyframe_received - start_id)
            list_index_range = range(s, end_id-start_id+1)
        self.update_received_kf_id(other_robot_id, max(self.neighbors_monitors[other_robot_id].last_keyframe_received, end_id))
        return list_index_range

        
        
       