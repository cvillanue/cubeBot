#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class RandomPolicy(Node):
    def __init__(self):
        super().__init__('random_policy')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.declare_parameter('hz', 5.0)
        self.declare_parameter('v_max', 0.6)
        self.declare_parameter('w_max', 1.2)
        hz = float(self.get_parameter('hz').get_parameter_value().double_value)
        self.timer = self.create_timer(1.0/hz, self.tick)
        self.get_logger().info('RandomPolicy up. Publishing random /cmd_vel')

    def tick(self):
        v_max = float(self.get_parameter('v_max').get_parameter_value().double_value)
        w_max = float(self.get_parameter('w_max').get_parameter_value().double_value)
        msg = Twist()
        msg.linear.x = random.uniform(0.0, v_max)   # forward only
        msg.angular.z = random.uniform(-w_max, w_max)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = RandomPolicy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
