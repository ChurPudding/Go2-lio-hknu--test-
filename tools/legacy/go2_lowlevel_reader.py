#!/usr/bin/env python3
"""
Go2 저수준(Low-level) 상태 리더
================================
`/lowstate` (unitree_go/msg/LowState) 토픽을 구독하여
- 12개 관절의 움직임 값: q(각도), dq(각속도), ddq(각가속도), tau_est(추정 토크)
- 본체 IMU: 쿼터니언, 각속도, 가속도, rpy
를 실시간 출력하고, 원하면 CSV로 기록합니다.

이 노드는 '읽기 전용'입니다. /lowcmd 로 명령을 보내지 않으므로 로봇은 움직이지 않습니다.

실행 예:
    python3 go2_lowlevel_reader.py            # 화면 출력만
    python3 go2_lowlevel_reader.py --csv       # 화면 출력 + CSV 저장
    python3 go2_lowlevel_reader.py --csv --rate 10   # 10Hz로만 출력/기록

종료: Ctrl+C
"""

import argparse
import csv
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_go.msg import LowState

# ---------------------------------------------------------------------------
# Go2 12관절 순서 (Unitree 공식 SDK 기준: FR -> FL -> RR -> RL, 각 다리 hip/thigh/calf)
# 주의: 펌웨어/버전에 따라 다를 수 있으므로, 한 관절만 손으로 움직여 보고
#       어떤 인덱스의 q 값이 변하는지 반드시 실측으로 검증하시길 권장합니다.
# ---------------------------------------------------------------------------
JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",   # 0, 1, 2  (오른쪽 앞다리)
    "FL_hip", "FL_thigh", "FL_calf",   # 3, 4, 5  (왼쪽 앞다리)
    "RR_hip", "RR_thigh", "RR_calf",   # 6, 7, 8  (오른쪽 뒷다리)
    "RL_hip", "RL_thigh", "RL_calf",   # 9, 10,11 (왼쪽 뒷다리)
]
NUM_JOINTS = 12


class LowLevelReader(Node):
    def __init__(self, use_csv: bool, rate_hz: float):
        super().__init__("go2_lowlevel_reader")

        # Go2 상태 토픽은 보통 BEST_EFFORT로 발행됩니다.
        # 구독자를 BEST_EFFORT로 두면 RELIABLE/BEST_EFFORT 어느 쪽이든 수신 가능합니다.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            LowState, "/lowstate", self.callback, qos
        )

        self.rate_hz = rate_hz
        self.min_interval = (1.0 / rate_hz) if rate_hz > 0 else 0.0
        self._last_print = 0.0

        # CSV 준비
        self.csv_writer = None
        self.csv_file = None
        if use_csv:
            fname = f"go2_lowstate_{datetime.now():%m%d_%H%M%S}.csv"
            self.csv_file = open(fname, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            header = ["t"]
            for name in JOINT_NAMES:
                header += [f"{name}_q", f"{name}_dq", f"{name}_ddq", f"{name}_tau"]
            header += ["quat_w", "quat_x", "quat_y", "quat_z",
                       "gyro_x", "gyro_y", "gyro_z",
                       "acc_x", "acc_y", "acc_z",
                       "roll", "pitch", "yaw"]
            self.csv_writer.writerow(header)
            self.get_logger().info(f"CSV 기록 시작: {fname}")

        self.get_logger().info("저수준 리더 시작. /lowstate 수신 대기 중...")

    def callback(self, msg: LowState):
        now = time.time()
        # 지정한 rate 보다 자주 오면 건너뜀 (rate<=0 이면 모두 처리)
        if self.min_interval > 0 and (now - self._last_print) < self.min_interval:
            return
        self._last_print = now

        # --- 관절 값 추출 (앞 12개만이 다리 관절) ---
        rows = []
        for i in range(NUM_JOINTS):
            m = msg.motor_state[i]
            rows.append((m.q, m.dq, m.ddq, m.tau_est))

        # --- IMU 추출 ---
        imu = msg.imu_state
        quat = imu.quaternion       # [w, x, y, z]
        gyro = imu.gyroscope        # [x, y, z] rad/s
        acc = imu.accelerometer     # [x, y, z] m/s^2
        rpy = imu.rpy               # [roll, pitch, yaw] rad

        # --- 화면 출력 ---
        print("\n" + "=" * 68)
        print(f"[저수준] {datetime.now():%H:%M:%S.%f}"[:-3])
        print(f"{'관절':<10}{'q(rad)':>10}{'dq(rad/s)':>12}"
              f"{'ddq':>10}{'tau(N·m)':>12}")
        print("-" * 68)
        for name, (q, dq, ddq, tau) in zip(JOINT_NAMES, rows):
            print(f"{name:<10}{q:>10.3f}{dq:>12.3f}{ddq:>10.2f}{tau:>12.2f}")
        print("-" * 68)
        print(f"IMU rpy(rad): roll={rpy[0]:+.3f}  pitch={rpy[1]:+.3f}  yaw={rpy[2]:+.3f}")

        # --- CSV 기록 ---
        if self.csv_writer:
            row = [f"{now:.4f}"]
            for (q, dq, ddq, tau) in rows:
                row += [f"{q:.5f}", f"{dq:.5f}", f"{ddq:.4f}", f"{tau:.4f}"]
            row += [f"{quat[0]:.5f}", f"{quat[1]:.5f}", f"{quat[2]:.5f}", f"{quat[3]:.5f}"]
            row += [f"{gyro[0]:.5f}", f"{gyro[1]:.5f}", f"{gyro[2]:.5f}"]
            row += [f"{acc[0]:.5f}", f"{acc[1]:.5f}", f"{acc[2]:.5f}"]
            row += [f"{rpy[0]:.5f}", f"{rpy[1]:.5f}", f"{rpy[2]:.5f}"]
            self.csv_writer.writerow(row)

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("CSV 파일을 저장하고 닫았습니다.")
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="Go2 저수준 상태 리더")
    parser.add_argument("--csv", action="store_true", help="CSV로 기록")
    parser.add_argument("--rate", type=float, default=0.0,
                        help="출력/기록 최대 주파수(Hz). 0이면 들어오는 대로 모두")
    args = parser.parse_args()

    rclpy.init()
    node = LowLevelReader(use_csv=args.csv, rate_hz=args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
