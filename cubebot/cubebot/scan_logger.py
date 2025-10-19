#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv, time, os

class ScanLogger(Node):
    def __init__(self):
        super().__init__('scan_logger')
        self.declare_parameter('outfile', 'scan_log.csv')
        self.outfile = self.get_parameter('outfile').get_parameter_value().string_value
        self.sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        # Write header
        self._ensure_header_written = False
        self.get_logger().info(f'Logging /scan to {self.outfile}')

    def on_scan(self, msg: LaserScan):
        fn = self.outfile
        os.makedirs(os.path.dirname(fn) or '.', exist_ok=True)
        with open(fn, 'a', newline='') as f:
            w = csv.writer(f)
            if not self._ensure_header_written:
                w.writerow(['t', 'angle_min', 'angle_max', 'angle_increment', 'range_count'])
                self._ensure_header_written = True
            w.writerow([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9, msg.angle_min, msg.angle_max, msg.angle_increment, len(msg.ranges)])

def main():
    rclpy.init()
    node = ScanLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
