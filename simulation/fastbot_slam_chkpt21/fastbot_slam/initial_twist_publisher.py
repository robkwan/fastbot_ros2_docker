# fastbot_slam/initial_twist_publisher.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class InitialTwistPublisher(Node):
    def __init__(self):
        super().__init__('initial_twist_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.duration = 2.0      # total rotation duration in seconds
        self.elapsed = 0.0
        self.timer = self.create_timer(self.timer_period, self.publish_twist)

    def publish_twist(self):
        twist = Twist()

        if self.elapsed < self.duration:
            # Send small rotation command
            twist.angular.z = 0.2
            self.get_logger().info('Publishing initial rotation - step 1')
        #else:
            # Stop the robot after initial motion
        #    twist.angular.z = 0.0
        #    self.publisher.publish(twist)
        #    self.get_logger().info('Stopping robot. Exiting node.')
        #    self.destroy_timer(self.timer)
        #    rclpy.shutdown()
        #    return

        self.publisher.publish(twist)
        self.elapsed += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = InitialTwistPublisher()
    rclpy.spin(node)
