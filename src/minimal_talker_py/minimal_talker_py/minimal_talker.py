import rclpy
from rclpy.node import Node

class MinimalTalker(Node):
    def __init__(self):
        super().__init__('minimal_talker')
        self.get_logger().info('MinimalTalker started')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
