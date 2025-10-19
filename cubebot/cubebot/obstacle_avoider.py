#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        # Parameters
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('turn_speed', 0.8)
        self.declare_parameter('min_range', 0.6)
        self.declare_parameter('fov_deg', 60.0)  # forward field of view to check for obstacles
        self.get_logger().info('ObstacleAvoider up. Listening on /scan, publishing /cmd_vel')

    def on_scan(self, msg: LaserScan):
        lin = float(self.get_parameter('linear_speed').get_parameter_value().double_value)
        ang = float(self.get_parameter('turn_speed').get_parameter_value().double_value)
        min_r = float(self.get_parameter('min_range').get_parameter_value().double_value)
        fov = math.radians(float(self.get_parameter('fov_deg').get_parameter_value().double_value))

        # Compute indices for +/- fov/2 around the front (0 angle assumed straight ahead)
        # In Gazebo Classic + ROS, scan angle_min to angle_max increasing. We'll center near 0.
        # Find indices close to angle 0 within +/- fov/2
        angles = [msg.angle_min + i*msg.angle_increment for i in range(len(msg.ranges))]
        forward_indices = [i for i, a in enumerate(angles) if abs(a) <= fov/2.0]
        forward_ranges = [msg.ranges[i] for i in forward_indices if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i])]

        twist = Twist()
        if forward_ranges and min(forward_ranges) < min_r:
            # Obstacle ahead: turn away
            twist.linear.x = 0.0
            # Bias turn direction by comparing left/right halves
            left = [msg.ranges[i] for i, a in enumerate(angles) if 0.0 < a <= fov/2.0]
            right = [msg.ranges[i] for i, a in enumerate(angles) if -fov/2.0 <= a < 0.0]
            lmin = min([r for r in left if not math.isinf(r) and not math.isnan(r)], default=999.0)
            rmin = min([r for r in right if not math.isinf(r) and not math.isnan(r)], default=999.0)
            twist.angular.z = ang if lmin < rmin else -ang
        else:
            # Clear path
            twist.linear.x = lin
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
