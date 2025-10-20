#!/usr/bin/env python3
import rclpy, time, math, csv, os
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RLLite(Node):
    def __init__(self):
        super().__init__('rl_lite')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.timer = self.create_timer(0.1, self.step)  # 10 Hz
        # params
        self.declare_parameter('episode_len', 600)   # steps (~60s)
        self.declare_parameter('min_safe', 0.22)     # collision threshold
        self.declare_parameter('log_csv', 'rl_runs/rl_lite_log.csv')
        self.step_i = 0
        self.ep_ret = 0.0
        self.min_r = 5.0
        self.last_scan = None
        os.makedirs(os.path.dirname(self.get_parameter('log_csv').value), exist_ok=True)
        self.csv = open(self.get_parameter('log_csv').value, 'a', newline='')
        self.writer = csv.writer(self.csv)
        if self.csv.tell() == 0:
            self.writer.writerow(['episode','step','reward','min_range','v','w'])

        self.episode = 1
        self.get_logger().info('RL-lite started. Publishing to /cmd_vel, reading /scan.')

    def on_scan(self, msg: LaserScan):
        rngs = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        self.min_r = min(rngs) if rngs else 5.0
        self.last_scan = msg

    def step(self):
        if self.last_scan is None:  # wait for first scan
            return

        # very simple "policy": go forward; steer away from closer side
        v_max, w_max = 0.6, 1.2
        fov = math.radians(70)
        msg = self.last_scan
        angles = [msg.angle_min + i*msg.angle_increment for i in range(len(msg.ranges))]
        left = [msg.ranges[i] for i,a in enumerate(angles) if 0.0 < a <= fov/2]
        right= [msg.ranges[i] for i,a in enumerate(angles) if -fov/2 <= a < 0.0]
        lmin = min([r for r in left if 0.05 < r < 10], default=5.0)
        rmin = min([r for r in right if 0.05 < r < 10], default=5.0)

        # action
        v = max(0.0, min(v_max, (self.min_r - 0.25)))   # slow when close
        w = (rmin - lmin)                                # steer to side with more space
        w = max(-w_max, min(w_max, 2.0*w))

        # reward: forward progress minus turning and proximity penalties
        reward = v - 0.02*abs(w) - (0.5 if self.min_r < 0.22 else 0.0)

        # publish
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub.publish(twist)

        # log
        self.step_i += 1
        self.ep_ret += reward
        self.writer.writerow([self.episode, self.step_i, reward, self.min_r, v, w])

        # episode end on collision or timeout
        if self.min_r < float(self.get_parameter('min_safe').value) or \
           self.step_i >= int(self.get_parameter('episode_len').value):
            self.get_logger().info(f'EP {self.episode} done: return={self.ep_ret:.3f}, steps={self.step_i}')
            self.episode += 1
            self.step_i, self.ep_ret = 0, 0.0

def main():
    rclpy.init()
    n = RLLite()
    try: rclpy.spin(n)
    finally:
        n.csv.close()
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
