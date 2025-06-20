import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StarterNode(Node):
    def __init__(self):
        super().__init__('starter_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_velocity)
        self.get_logger().info('Starter node publishing forward velocity...')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5   # Move forward
        msg.angular.z = 0.0  # No rotation
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StarterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
