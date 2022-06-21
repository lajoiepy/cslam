from test_msgs.msg import Empty as EmptyMsg
from rclpy.duration import Duration

class NeighborMonitor():
    """Monitors if a neighboring robot is in range
    """
    def __init__(self, node, id, max_delay_sec):
        """Initialization
        Args:
            id (int): Robot ID
        """
        self.node = node
        self.id  = id
        self.max_delay_sec = Duration(seconds=max_delay_sec)
        self.alive = False
        self.init_time =  self.node.get_clock().now()
        self.latest_time_stamp = self.init_time
                   
        self.alive_subscriber = self.node.create_subscription(
            EmptyMsg, '/r' + str(id) + '/' + 'alive', self.alive_callback, 10)

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
        now = self.node.get_clock().now()
        return now - self.init_time > self.max_delay_sec and now - self.latest_time_stamp < self.max_delay_sec