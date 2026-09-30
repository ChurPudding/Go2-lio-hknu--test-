#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class ZVD(Node):
    def __init__(self):
        super().__init__('zvd_node')
        self.declare_parameter('v_th', 0.05)  # m/s 정지 판정
        self.v_th = float(self.get_parameter('v_th').value)
        self.declare_parameter('dwell', 0.25)  # s 유지
        self.dwell = float(self.get_parameter('dwell').value)
        self.declare_parameter('release', 1.5)  # 해제 배수(히스테리시스)
        self.release = float(self.get_parameter('release').value)
        self.pub = self.create_publisher(Bool, '/zupt_active', 10)
        self.sub = self.create_subscription(Odometry, '/utlidar/robot_odom', self.cb, 20)
        self.still_since = None
        self.active = False

    def cb(self, msg):
        v = msg.twist.twist.linear
        speed = (v.x**2 + v.y**2 + v.z**2) ** 0.5
        t = self.get_clock().now().nanoseconds * 1e-9
        if not self.active:
            if speed < self.v_th:
                if self.still_since is None:
                    self.still_since = t
                elif t - self.still_since >= self.dwell:
                    self.active = True
            else:
                self.still_since = None
        else:
            if speed > self.v_th * self.release:
                self.active = False
                self.still_since = None
        m = Bool(); m.data = self.active
        self.pub.publish(m)

def main():
    rclpy.init(); n = ZVD(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
