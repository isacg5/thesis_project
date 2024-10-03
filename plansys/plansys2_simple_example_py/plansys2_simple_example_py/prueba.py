#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdomAdjuster(Node):
    def __init__(self):
        super().__init__('odom_adjuster')
        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Odometry, '/adjusted_odom', 10)
        self.initial_x = -2.00
        self.initial_y = -0.5

    def odom_callback(self, msg): 
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Adjust the odometry
        adjusted_msg = msg
        adjusted_msg.pose.pose.position.x = x - self.initial_x
        adjusted_msg.pose.pose.position.y = y - self.initial_y

        # Publish the adjusted odometry
        self.publisher.publish(adjusted_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomAdjuster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
