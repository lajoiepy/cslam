from std_msgs.msg import UInt32
from time import time

class NeighborMonitor():
    """Monitors if a neighboring robot is in range
    """

    def __init__(self, node, rid, is_enabled, init_delay_sec, max_delay_sec):
        """Initialization
        Args:
            id (int): Robot ID
        """
        self.node = node
        self.robot_id = rid
        self.is_enabled = is_enabled
        self.origin_robot_id = self.robot_id

        self.init_delay_sec = init_delay_sec
        self.max_delay_sec = max_delay_sec
        self.first_heartbeat_received = False
        self.init_time = time()
        self.latest_time_stamp = self.init_time
        self.last_keyframe_received = -1
        self.last_keyframe_sent = -1
        self.last_match_sent = -1

        self.heartbeat_subscriber = self.node.create_subscription(
            UInt32, '/r' + str(rid) + '/' + 'cslam/heartbeat',
            self.heartbeat_callback, 10)

    def heartbeat_callback(self, msg):
        """Callback to indicate that it is alive

        Args:
            msg (UInt32):
        """
        self.origin_robot_id = msg.data
        self.latest_time_stamp = time()
        if not self.first_heartbeat_received:
            self.first_heartbeat_received = True
            self.init_time = time()

    def is_alive(self):
        """Check if it recently received a heartbeat signal

        Returns:
            bool: liveliness indicator
        """
        if self.is_enabled:
            now = time()
            return self.first_heartbeat_received and now - self.init_time > self.init_delay_sec and now - self.latest_time_stamp < self.max_delay_sec
        else:
            True
