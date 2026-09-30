#!/usr/bin/env python3
"""다리 오도메트리 파라미터 스윕용 '원시' 기록 노드.

bag 을 딱 한 번 재생하면서, odom 샘플마다 그 순간의 원시 입력을 통째로 남긴다:
  t, q0..q11, dq0..dq11, tau0..tau11, wx,wy,wz, vox,voy,voz, omx,omy
접지 판정/평균/필터를 전혀 안 하고 원자료만 저장 → force_thr·alpha 를
오프라인(sweep_leg_odom.py)에서 재생 없이 무한정 바꿔볼 수 있다.

  export ROS_DOMAIN_ID=99
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
  unset CYCLONEDDS_URI
  python3 record_raw_leg.py            # 터미널 A
  ros2 bag play ~/data/bags/<bag>      # 터미널 B, 끝나면 A 에서 Ctrl-C
"""

import csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_go.msg import LowState
from nav_msgs.msg import Odometry


class RawRecorder(Node):
    def __init__(self):
        super().__init__("record_raw_leg")
        self.path = self.declare_parameter("csv_path", "leg_raw.csv").value
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LowState, "lowstate", self.on_low, be)
        self.create_subscription(Odometry, "/utlidar/robot_odom", self.on_odom, 10)
        self.low = None
        self.csv = open(self.path, "w", newline="")
        self.w = csv.writer(self.csv)
        hdr = (["t"] + [f"q{i}" for i in range(12)] + [f"dq{i}" for i in range(12)]
               + [f"tau{i}" for i in range(12)] + ["wx", "wy", "wz"]
               + ["vox", "voy", "voz", "omx", "omy", "wox", "woy", "woz"])
        self.w.writerow(hdr)
        self.n = 0
        self.get_logger().info(f"기록 시작 → {self.path}. bag 재생 후 Ctrl-C.")

    def on_low(self, msg: LowState):
        q = [msg.motor_state[i].q for i in range(12)]
        dq = [msg.motor_state[i].dq for i in range(12)]
        tau = [msg.motor_state[i].tau_est for i in range(12)]
        w = list(msg.imu_state.gyroscope)
        self.low = q + dq + tau + w

    def on_odom(self, msg: Odometry):
        if self.low is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tw = msg.twist.twist.linear
        ta = msg.twist.twist.angular
        p = msg.pose.pose.position
        self.w.writerow([f"{t:.4f}"] + [f"{x:.5f}" for x in self.low]
                        + [f"{tw.x:.5f}", f"{tw.y:.5f}", f"{tw.z:.5f}",
                           f"{p.x:.4f}", f"{p.y:.4f}",
                           f"{ta.x:.5f}", f"{ta.y:.5f}", f"{ta.z:.5f}"])
        self.n += 1

    def destroy_node(self):
        try:
            self.csv.close()
            self.get_logger().info(f"저장 완료: {self.n} 행 → {self.path}")
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = RawRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
