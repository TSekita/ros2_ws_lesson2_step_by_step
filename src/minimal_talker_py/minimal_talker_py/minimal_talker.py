# 回答
# ~/ros2_ws/src/minimal_talker_py/minimal_talker_py/minimal_talker.pyを作成
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MinimalTalker(Node):
    def __init__(self):
        super().__init__('minimal_talker')
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE
        )
        """
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE
        )
        """
        # qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
        # self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # timer_period = 0.5 # seconds
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Count: %d' % self.count
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Count: {self.count}')
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
