#!/usr/bin/env python3
"""
Go2 고수준(High-level) 상태 / 오도메트리 리더
============================================
두 가지 소스에서 오도메트리를 읽습니다.

1) `/sportmodestate` (unitree_go/msg/SportModeState)
   - position[3]  : 본체 위치 (x, y, z)  <- 내부 상태추정기가 만든 오도메트리
   - velocity[3]  : 본체 속도 (vx, vy, vz)
   - yaw_speed    : 요(yaw) 각속도
   - body_height  : 몸통 높이
   - foot_position_body[12], foot_speed_body[12] : 발 4개의 위치/속도 (몸통 기준)
   - mode, gait_type : 현재 운동 모드 / 걸음새

2) `/utlidar/robot_odom` (nav_msgs/msg/Odometry)
   - 표준 ROS2 Odometry 형식 (pose + twist). RViz/Nav2 등과 바로 호환.

이 노드는 '읽기 전용'입니다. 로봇에 명령을 보내지 않습니다.

실행 예:
    python3 go2_highlevel_reader.py            # 화면 출력만
    python3 go2_highlevel_reader.py --csv       # CSV 저장
    python3 go2_highlevel_reader.py --csv --rate 10

종료: Ctrl+C
"""

import argparse
import csv
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_go.msg import SportModeState
from nav_msgs.msg import Odometry

FOOT_NAMES = ["FR", "FL", "RR", "RL"]  # 발 순서 (관절 순서와 동일 계열)


class HighLevelReader(Node):
    def __init__(self, use_csv: bool, rate_hz: float):
        super().__init__("go2_highlevel_reader")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_sport = self.create_subscription(
            SportModeState, "/sportmodestate", self.sport_cb, qos
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/utlidar/robot_odom", self.odom_cb, qos
        )

        self.rate_hz = rate_hz
        self.min_interval = (1.0 / rate_hz) if rate_hz > 0 else 0.0
        self._last_print = 0.0

        # 마지막으로 받은 표준 Odometry 를 보관 (sport 콜백에서 함께 출력)
        self._last_odom = None

        # CSV 준비
        self.csv_writer = None
        self.csv_file = None
        if use_csv:
            fname = f"go2_odom_{datetime.now():%m%d_%H%M%S}.csv"
            self.csv_file = open(fname, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            header = ["t",
                      "pos_x", "pos_y", "pos_z",
                      "vel_x", "vel_y", "vel_z", "yaw_speed",
                      "body_height", "mode", "gait_type"]
            for f in FOOT_NAMES:
                header += [f"{f}_foot_x", f"{f}_foot_y", f"{f}_foot_z"]
            # 표준 odom 도 함께 기록
            header += ["odom_x", "odom_y", "odom_z",
                       "odom_qx", "odom_qy", "odom_qz", "odom_qw"]
            self.csv_writer.writerow(header)
            self.get_logger().info(f"CSV 기록 시작: {fname}")

        self.get_logger().info("고수준 리더 시작. /sportmodestate 수신 대기 중...")

    def odom_cb(self, msg: Odometry):
        # 표준 Odometry 는 값만 저장해 두고, sport 콜백에서 함께 출력/기록
        self._last_odom = msg

    def sport_cb(self, msg: SportModeState):
        now = time.time()
        if self.min_interval > 0 and (now - self._last_print) < self.min_interval:
            return
        self._last_print = now

        pos = msg.position       # [x, y, z]
        vel = msg.velocity       # [vx, vy, vz]
        yaw_speed = msg.yaw_speed
        body_h = msg.body_height
        mode = msg.mode
        gait = msg.gait_type

        # 발 위치 (12개 = 4발 x 3축)
        fpb = msg.foot_position_body

        # --- 화면 출력 ---
        print("\n" + "=" * 60)
        print(f"[고수준] {datetime.now():%H:%M:%S.%f}"[:-3]
              + f"   mode={mode}  gait={gait}")
        print("-" * 60)
        print(f"위치(m)     x={pos[0]:+.3f}  y={pos[1]:+.3f}  z={pos[2]:+.3f}")
        print(f"속도(m/s)   vx={vel[0]:+.3f} vy={vel[1]:+.3f} vz={vel[2]:+.3f}")
        print(f"요각속도    yaw_speed={yaw_speed:+.3f} rad/s   몸통높이={body_h:.3f} m")
        print("-" * 60)
        print("발 위치 (몸통 기준, m):")
        for i, f in enumerate(FOOT_NAMES):
            x, y, z = fpb[i*3], fpb[i*3+1], fpb[i*3+2]
            print(f"  {f}: x={x:+.3f}  y={y:+.3f}  z={z:+.3f}")

        # 표준 Odometry 도 있으면 함께 표시
        odom = self._last_odom
        if odom is not None:
            p = odom.pose.pose.position
            print("-" * 60)
            print(f"[표준 odom] x={p.x:+.3f}  y={p.y:+.3f}  z={p.z:+.3f}")

        # --- CSV 기록 ---
        if self.csv_writer:
            row = [f"{now:.4f}",
                   f"{pos[0]:.5f}", f"{pos[1]:.5f}", f"{pos[2]:.5f}",
                   f"{vel[0]:.5f}", f"{vel[1]:.5f}", f"{vel[2]:.5f}",
                   f"{yaw_speed:.5f}", f"{body_h:.5f}", mode, gait]
            for i in range(4):
                row += [f"{fpb[i*3]:.5f}", f"{fpb[i*3+1]:.5f}", f"{fpb[i*3+2]:.5f}"]
            if odom is not None:
                p = odom.pose.pose.position
                o = odom.pose.pose.orientation
                row += [f"{p.x:.5f}", f"{p.y:.5f}", f"{p.z:.5f}",
                        f"{o.x:.5f}", f"{o.y:.5f}", f"{o.z:.5f}", f"{o.w:.5f}"]
            else:
                row += [""] * 7
            self.csv_writer.writerow(row)

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("CSV 파일을 저장하고 닫았습니다.")
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="Go2 고수준 상태/오도메트리 리더")
    parser.add_argument("--csv", action="store_true", help="CSV로 기록")
    parser.add_argument("--rate", type=float, default=0.0,
                        help="출력/기록 최대 주파수(Hz). 0이면 들어오는 대로 모두")
    args = parser.parse_args()

    rclpy.init()
    node = HighLevelReader(use_csv=args.csv, rate_hz=args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
