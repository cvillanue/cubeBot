#!/usr/bin/env python3
import csv, time, os, math
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RewardLogger(Node):
    def __init__(self):
        super().__init__('reward_logger')

        # ---------- params ----------
        self.declare_parameter('episode_len', 600)        # steps per episode
        self.declare_parameter('hz', 10.0)                # logging/control rate
        self.declare_parameter('min_safe', 0.22)          # collision threshold (m)
        self.declare_parameter('episodes_csv', 'rl_runs/episodes.csv')
        self.declare_parameter('steps_csv',    'rl_runs/steps.csv')  # optional, per-step

        self.episode_len = int(self.get_parameter('episode_len').value)
        self.hz = float(self.get_parameter('hz').value)
        self.min_safe = float(self.get_parameter('min_safe').value)

        # ---------- IO ----------
        self.last_cmd = Twist()
        self.last_scan = None
        self.min_r = 5.0

        self.sub_cmd  = self.create_subscription(Twist,      '/cmd_vel', self.on_cmd, 10)
        self.sub_scan = self.create_subscription(LaserScan,  '/scan',    self.on_scan, 10)

        # ---------- files ----------
        Path('rl_runs').mkdir(parents=True, exist_ok=True)
        self.ep_path = self.get_parameter('episodes_csv').value
        self.st_path = self.get_parameter('steps_csv').value

        self.ep_f = open(self.ep_path, 'a', newline='')
        self.ep_w = csv.writer(self.ep_f)
        if self.ep_f.tell() == 0:
            self.ep_w.writerow(['episode','episode_return','episode_length','terminal','timestamp'])

        self.st_f = open(self.st_path, 'a', newline='')
        self.st_w = csv.writer(self.st_f)
        if self.st_f.tell() == 0:
            self.st_w.writerow(['episode','step','reward','min_range','v','w','timestamp'])

        # ---------- episode state ----------
        self.episode = 1
        self.step_i = 0
        self.ep_ret = 0.0

        # tick
        self.timer = self.create_timer(1.0/self.hz, self.tick)
        self.get_logger().info('RewardLogger ready: listening to /cmd_vel and /scan')

    # --------- callbacks ----------
    def on_cmd(self, msg: Twist):
        self.last_cmd = msg

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        rngs = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        self.min_r = min(rngs) if rngs else 5.0

    # --------- reward function (edit if you like) ----------
    def compute_reward(self, v, w, min_r):
        # encourage forward motion, discourage sharp turns, penalize proximity/collisions
        r = v - 0.02*abs(w) - (0.5 if min_r < self.min_safe else 0.0)
        return r

    # --------- main loop ----------
    def tick(self):
        if self.last_scan is None:
            return

        v = float(self.last_cmd.linear.x)
        w = float(self.last_cmd.angular.z)
        r = self.compute_reward(v, w, self.min_r)

        self.step_i += 1
        self.ep_ret += r

        # optional per-step line (useful for debugging)
        self.st_w.writerow([self.episode, self.step_i, r, self.min_r, v, w, time.time()])
        if self.step_i % 25 == 0:  # flush occasionally
            self.st_f.flush()

        # termination conditions: collision or timeout
        terminal = None
        if self.min_r < self.min_safe:
            terminal = 'collision'
        elif self.step_i >= self.episode_len:
            terminal = 'timeout'

        if terminal:
            self.get_logger().info(f'EP {self.episode} done: return={self.ep_ret:.3f}, steps={self.step_i}, term={terminal}')
            self.ep_w.writerow([self.episode, round(self.ep_ret, 6), self.step_i, terminal, time.time()])
            self.ep_f.flush()

            # reset counters
            self.episode += 1
            self.step_i = 0
            self.ep_ret = 0.0

    def destroy_node(self):
        try:
            self.ep_f.close()
            self.st_f.close()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = RewardLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
