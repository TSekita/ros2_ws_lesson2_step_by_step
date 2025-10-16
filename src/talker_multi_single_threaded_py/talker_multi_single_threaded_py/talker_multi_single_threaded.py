import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from std_msgs.msg import String
import time

class FastPublisher(Node):
    def __init__(self):
        super().__init__('fast_pub')
        self.pub = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.1, self.publish_fast)

    def publish_fast(self):
        msg = String()
        msg.data = f"fast {time.time()}"
        self.pub.publish(msg)
        self.get_logger().info('Fast')

class SlowSubscriber(Node):
    def __init__(self):
        super().__init__('slow_sub')
        self.sub = self.create_subscription(String, 'topic', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        time.sleep(2)
        self.get_logger().info("Done")

def main():
    rclpy.init()
    fast = FastPublisher()
    slow = SlowSubscriber()

    # Single-threaded
    # executor = SingleThreadedExecutor()

    # Multi-threaded
    executor = MultiThreadedExecutor(num_threads=2)

    executor.add_node(fast)
    executor.add_node(slow)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        fast.destroy_node()
        slow.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
