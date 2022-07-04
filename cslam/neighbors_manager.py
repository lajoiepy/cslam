from cslam.neighbor_monitor import NeighborMonitor
from cslam_common_interfaces.msg import RobotIds
from std_msgs.msg import String

class NeighborManager():
    def __init__(self, node, robot_id, nb_robots, is_enabled,  max_delay_sec):
        self.node = node
        self.robot_id = robot_id
        self.nb_robots = nb_robots
        self.neighbors_monitors = {}
        for id in range(self.nb_robots):
            if id != self.robot_id:
                self.neighbors_monitors[id] = NeighborMonitor(self.node, id, is_enabled, max_delay_sec)
        
        self.subscriber = self.node.create_subscription(
            String, 'get_current_neighbors', self.get_current_neighbors_callback, 100) 
        self.neighbors_publisher = self.node.create_publisher(
            RobotIds, 'current_neighbors', 100)

    def check_neighbors_in_range(self):
        """Check which neighbors are in range
        
        """
        is_robot_in_range = {}
        robots_in_range_list = []
        for i in range(self.nb_robots):
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
        for i in range(self.nb_robots):
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
        for i in range(self.nb_robots):
            if i != self.robot_id:
                if self.neighbors_monitors[i].is_alive():
                    from_kf_id = min(self.neighbors_monitors[i].last_keyframe_sent, from_kf_id)
                    
        for i in range(self.nb_robots):
            if i != self.robot_id:
                if self.neighbors_monitors[i].is_alive():
                    self.neighbors_monitors[i].last_keyframe_sent = latest_local_id

        return from_kf_id + 1

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
            list_index_range = range(0)
        else:
            s = max(0, self.neighbors_monitors[other_robot_id].last_keyframe_received - start_id)
            list_index_range = range(s, end_id-start_id+1)
        self.update_received_kf_id(other_robot_id, max(self.neighbors_monitors[other_robot_id].last_keyframe_received, end_id))
        return list_index_range

    def get_current_neighbors_callback(self, msg):
        """Publish the current neighbors in range

        Args:
            msg (String): Empty
        """
        is_robot_in_range, robots_in_range_list = self.check_neighbors_in_range()
        msg = RobotIds()
        msg.ids = robots_in_range_list
        self.neighbors_publisher.publish(msg)
        
       