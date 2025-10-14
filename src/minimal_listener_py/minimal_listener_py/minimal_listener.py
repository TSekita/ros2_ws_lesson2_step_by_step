# ~/ros2_ws/src/minimal_listener_py/minimal_listener_py/minimal_listener.py
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MinimalListener(Node):
    def __init__(self):
        super().__init__('minimal_listener')
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(String, 'chatter', self.listener, qos_profile)
        # self.subscription = self.create_subscription(
        #    String,
        #    'chatter',
        #    self.listener,
        #    10)
        self.subscription

    def listener(self, msg):
        self.get_logger().info('I heard "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MinimalListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
