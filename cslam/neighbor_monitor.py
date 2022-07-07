from std_msgs.msg import UInt32
from rclpy.duration import Duration

class NeighborMonitor():
    """Monitors if a neighboring robot is in range
    """
    def __init__(self, node, id, is_enabled, max_delay_sec):
        """Initialization
        Args:
            id (int): Robot ID
        """
        self.node = node
        self.robot_id  = id
        self.is_enabled = is_enabled
        self.origin_robot_id = self.robot_id

        self.max_delay_sec = Duration(seconds=max_delay_sec)
        self.heartbeat = False
        self.init_time =  self.node.get_clock().now()
        self.latest_time_stamp = self.init_time
        self.last_keyframe_received = -1
        self.last_keyframe_sent = -1
                   
        self.heartbeat_subscriber = self.node.create_subscription(
            UInt32, '/r' + str(id) + '/' + 'heartbeat', self.heartbeat_callback, 10)

    def heartbeat_callback(self, msg):
        """Callback to indicate that it is heartbeat

        Args:
            msg (UInt32):
        """
        self.origin_robot_id = msg.data
        self.latest_time_stamp = self.node.get_clock().now()

    def is_alive(self):
        """Check if it recently received a heartbeat signal

        Returns:
            bool: liveliness indicator
        """
        if self.is_enabled:
            now = self.node.get_clock().now()
            return now - self.init_time > self.max_delay_sec and now - self.latest_time_stamp < self.max_delay_sec
        else:
            True