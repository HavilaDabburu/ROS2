import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class StopperNode(Node):
    def __init__(self):
        super().__init__('stopper_node')

        self.threshold = 1.1  # stop if obstacle is within 1.1 meters
        self.min_distance = float('inf')  # updated by LIDAR
        self.latest_cmd_raw = Twist()

        # Subscribe to LIDAR
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)

        # Subscribe to teleop (remapped to /cmd_vel_raw)
        self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_callback, 10)

        # Publisher to filtered velocity topic
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer to publish filtered cmd
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def lidar_callback(self, msg):
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max and r > 0.1
        ]

        if not valid_ranges:
            self.get_logger().warn('No valid LIDAR data!')
            return

        self.min_distance = min(valid_ranges)
        self.get_logger().info(f"Minimum distance: {self.min_distance:.2f} m")

    def cmd_callback(self, msg):
        self.latest_cmd_raw = msg
        self.get_logger().info(f"Received cmd_vel_raw: linear.x = {msg.linear.x:.2f}")

    def publish_cmd(self):
        safe_cmd = Twist()
        safe_cmd.linear = self.latest_cmd_raw.linear
        safe_cmd.angular = self.latest_cmd_raw.angular

        if self.min_distance < self.threshold and self.latest_cmd_raw.linear.x > 0.0:
            self.get_logger().info("Obstacle ahead! Blocking forward motion.")
            safe_cmd.linear.x = 0.0

        self.get_logger().info(f"Publishing safe cmd_vel: linear.x = {safe_cmd.linear.x:.2f}")
        self.pub.publish(safe_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StopperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
