from std_msgs.msg import String
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
        self.id  = id
        self.is_enabled = is_enabled

        self.max_delay_sec = Duration(seconds=max_delay_sec)
        self.alive = False
        self.init_time =  self.node.get_clock().now()
        self.latest_time_stamp = self.init_time
        self.last_keyframe_received = 0
        self.last_keyframe_sent = 0
                   
        self.alive_subscriber = self.node.create_subscription(
            String, '/r' + str(id) + '_' + 'alive', self.alive_callback, 10)

    def alive_callback(self, msg):
        """Callback to indicate that it is alive

        Args:
            msg (Empty):
        """
        self.latest_time_stamp = self.node.get_clock().now()

    def is_alive(self):
        """Check if it recently received a alive signal

        Returns:
            bool: liveliness indicator
        """
        if self.is_enabled:
            now = self.node.get_clock().now()
            return now - self.init_time > self.max_delay_sec and now - self.latest_time_stamp < self.max_delay_sec
        else:
            True