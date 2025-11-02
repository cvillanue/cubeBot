#!/usr/bin/env python3
import rclpy, random, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

ACTIONS = [
    (0.00,  0.00),  # Stop
    (0.25,  0.00),  # Forward
    (0.20, +0.60),  # ForwardLeft
    (0.20, -0.60),  # ForwardRight
    (0.00, +0.80),  # InPlaceTurn (we'll flip sign randomly below)
]
PROBS = [0.10, 0.40, 0.20, 0.20, 0.10]   # must sum to 1.0

class RandomPolicyShielded(Node):
    def __init__(self):
        super().__init__('random_policy')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

        self.declare_parameter('hz', 10.0)
        self.declare_parameter('action_hold_s', 0.5)   # ~5 ticks at 10 Hz
        self.declare_parameter('min_safe', 0.18)

        self.min_r = 5.0
        self.ticks_left = 0
        self.curr_action = (0.0, 0.0)

        hz = float(self.get_parameter('hz').value)
        self.timer = self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info('RandomPolicy (shielded) publishing to /cmd_vel')

    def on_scan(self, msg: LaserScan):
        vals = [r for r in msg.ranges if math.isfinite(r) and 0.14 < r < msg.range_max]
        self.min_r = min(vals) if vals else msg.range_max

    def sample_action(self):
        a = random.choices(ACTIONS, weights=PROBS, k=1)[0]
        # Randomize in-place turn direction
        if a == ACTIONS[4]:
            a = (0.0, random.choice([+0.80, -0.80]))
        return a

    def tick(self):
        if self.ticks_left <= 0:
            self.curr_action = self.sample_action()
            hold = float(self.get_parameter('action_hold_s').value)
            hz = float(self.get_parameter('hz').value)
            self.ticks_left = max(1, int(hold * hz))

        v, w = self.curr_action

        # Safety shield: if too close, back-and-turn
        min_safe = float(self.get_parameter('min_safe').value)
        if self.min_r < min_safe:
            v, w = -0.05, random.choice([+0.8, -0.8])
            # Optionally shorten remaining hold to react faster
            self.ticks_left = min(self.ticks_left, 2)

        # Publish
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.pub.publish(msg)
        self.ticks_left -= 1

def main():
    rclpy.init()
    node = RandomPolicyShielded()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
