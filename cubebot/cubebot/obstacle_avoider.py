#!/usr/bin/env python3
import rclpy, math, time, statistics as st
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

DRIVE, AVOID, RECOVER = 0, 1, 2

class Avoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # pubs/subs
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        # params (tune freely)
        self.declare_parameter('hz', 10.0)
        self.declare_parameter('v_fwd', 0.18)
        self.declare_parameter('w_gain', 2.0)          # steer strength from left/right diff
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('min_safe', 0.22)       # collision threshold
        self.declare_parameter('front_safe', 0.35)     # “too close ahead” threshold
        self.declare_parameter('recover_secs', 0.8)    # reverse arc duration when stuck
        self.declare_parameter('stuck_eps', 0.03)      # meters of motion considered “no progress”
        self.declare_parameter('stuck_ticks', 12)

        # state
        self.state = DRIVE
        self.scan = None
        self.min_r = 5.0
        self.front_min = 5.0
        self.left_min = 5.0
        self.right_min = 5.0

        self.last_pose = None
        self.no_move_count = 0
        self.recover_until = 0.0
        self.recover_turn = +1.0  # flip each time

        hz = float(self.get_parameter('hz').value)
        self.timer = self.create_timer(1.0/hz, self.tick)
        self.get_logger().info('Obstacle avoider with stuck recovery up.')

    # ---------- helpers ----------
    @staticmethod
    def yaw_from_quat(q):
        # z-yaw from quaternion
        import math
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def on_odom(self, msg: Odometry):
        # track displacement to detect “stuck”
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_pose is None:
            self.last_pose = (x, y)
            return
        dx = (x - self.last_pose[0])
        dy = (y - self.last_pose[1])
        dist = (dx*dx + dy*dy) ** 0.5
        self.last_pose = (x, y)

        # if we’re trying to move forward and barely moved, count it
        stuck_eps = float(self.get_parameter('stuck_eps').value)
        if dist < stuck_eps:
            self.no_move_count += 1
        else:
            self.no_move_count = 0

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        # filter ranges
        rs = []
        angs = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r): 
                continue
            if r < msg.range_min + 0.02: 
                continue
            if r > msg.range_max - 1e-3:
                continue
            rs.append(r)
            angs.append(msg.angle_min + i*msg.angle_increment)

        # sectors (front ±30°, left 30–90°, right -90–-30°)
        def sector_min(lo_deg, hi_deg):
            lo = math.radians(lo_deg); hi = math.radians(hi_deg)
            vals = [r for r,a in zip(rs, angs) if lo <= a <= hi]
            return min(vals) if vals else 5.0

        self.front_min = sector_min(-30, 30)
        self.left_min  = sector_min( 10, 90)
        self.right_min = sector_min(-90,-10)
        self.min_r     = min(self.front_min, self.left_min, self.right_min)

    def tick(self):
        if self.scan is None:
            return

        v_fwd      = float(self.get_parameter('v_fwd').value)
        w_gain     = float(self.get_parameter('w_gain').value)
        turn_speed = float(self.get_parameter('turn_speed').value)
        min_safe   = float(self.get_parameter('min_safe').value)
        front_safe = float(self.get_parameter('front_safe').value)
        stuck_ticks= int(self.get_parameter('stuck_ticks').value)
        recover_s  = float(self.get_parameter('recover_secs').value)

        now = time.time()

        # ----- state transitions -----
        # hard collision guard
        if self.min_r < min_safe:
            self.state = RECOVER
            self.recover_until = now + recover_s
            self.recover_turn *= -1.0  # alternate turn direction each time

        # no progress while we *intended* to go forward → RECOVER
        elif self.state == DRIVE and self.no_move_count >= stuck_ticks:
            self.state = RECOVER
            self.recover_until = now + recover_s
            self.recover_turn *= -1.0
            self.no_move_count = 0

        # obstacle ahead but not touching → AVOID
        elif self.state in (DRIVE, AVOID) and self.front_min < front_safe:
            self.state = AVOID

        # clear ahead → DRIVE
        elif self.state == AVOID and self.front_min >= front_safe + 0.05:
            self.state = DRIVE

        # time out of recover → DRIVE
        if self.state == RECOVER and now >= self.recover_until:
            self.state = DRIVE

        # ----- action selection -----
        cmd = Twist()
        if self.state == DRIVE:
            # forward with steering away from closer side
            w = (self.right_min - self.left_min) * w_gain
            w = max(-1.2, min(1.2, w))
            cmd.linear.x = v_fwd
            cmd.angular.z = w

        elif self.state == AVOID:
            # stop and turn in place toward freer side
            turn = +turn_speed if self.right_min > self.left_min else -turn_speed
            cmd.linear.x = 0.0
            cmd.angular.z = turn

        else:  # RECOVER
            # gentle reverse arc
            cmd.linear.x = -0.08
            cmd.angular.z = self.recover_turn * 0.9

        self.pub.publish(cmd)

def main():
    rclpy.init()
    n = Avoider()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

