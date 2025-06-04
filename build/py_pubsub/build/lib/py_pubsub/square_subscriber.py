import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32

class SquareSubscriber(Node):
    def __init__(self):
        super().__init__('square_subscriber')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Int32,
            'number',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        squared = msg.data ** 2
        self.get_logger().info(f'Received: {msg.data}, Square: {squared}')

def main(args=None):
    rclpy.init(args=args)
    node = SquareSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
