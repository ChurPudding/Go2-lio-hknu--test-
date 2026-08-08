#!/usr/bin/env python3
"""
Go2 라이다 + 오도메트리 CSV 로거
===============================
실시간 시각화를 돌리는 동안, 아래 두 가지를 CSV로 기록합니다.

1) 오도메트리: /utlidar/robot_odom (nav_msgs/Odometry)
   -> go2_odom_log_<시각>.csv  (메시지 1건당 1행)
      열: t, x, y, z, qx, qy, qz, qw, vx, vy, vz, wz

2) 라이다 점군: /utlidar/cloud_deskewed (sensor_msgs/PointCloud2)
   -> go2_lidar_log_<시각>.csv  (점 1개당 1행)
      열: frame, t, x, y, z, intensity
   * cloud_deskewed 는 odom(세계) 좌표계 기준이라, 저장된 점은 이미 세계
     좌표입니다. MATLAB 에서 프레임을 그대로 누적하면 지도가 됩니다.

점군은 프레임당 점이 매우 많으므로, --every 로 몇 프레임마다 한 번씩만
저장할지 조절하세요. (예: --every 5 → 5프레임에 1번)

실행 예:
    python3 go2_csv_logger.py                 # 기본(모든 프레임 저장)
    python3 go2_csv_logger.py --every 5        # 5프레임마다 라이다 저장
    python3 go2_csv_logger.py --every 10 --outdir ~/go2_logs

종료: Ctrl+C  (파일이 저장되고 닫힙니다)
"""

import argparse
import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Go2CsvLogger(Node):
    def __init__(self, outdir: str, every: int):
        super().__init__("go2_csv_logger")

        os.makedirs(outdir, exist_ok=True)
        stamp = datetime.now().strftime("%m%d_%H%M%S")

        # --- 오도메트리 CSV ---
        self.odom_path = os.path.join(outdir, f"go2_odom_log_{stamp}.csv")
        self.odom_file = open(self.odom_path, "w", newline="")
        self.odom_writer = csv.writer(self.odom_file)
        self.odom_writer.writerow(
            ["t", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz", "wz"]
        )

        # --- 라이다 CSV ---
        self.lidar_path = os.path.join(outdir, f"go2_lidar_log_{stamp}.csv")
        self.lidar_file = open(self.lidar_path, "w", newline="")
        self.lidar_writer = csv.writer(self.lidar_file)
        self.lidar_writer.writerow(["frame", "t", "x", "y", "z", "intensity"])

        self.every = max(1, every)
        self.frame_idx = 0          # 수신한 라이다 프레임 번호(전체)
        self.saved_frame = 0        # 실제 저장한 프레임 번호

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/utlidar/robot_odom", self.odom_cb, qos
        )
        self.sub_lidar = self.create_subscription(
            PointCloud2, "/utlidar/cloud_deskewed", self.lidar_cb, qos
        )

        self.get_logger().info(f"오도메트리 -> {self.odom_path}")
        self.get_logger().info(f"라이다   -> {self.lidar_path}")
        self.get_logger().info(f"라이다 저장 주기: {self.every} 프레임마다 1회")
        self.get_logger().info("기록 시작. 종료하려면 Ctrl+C.")

    def odom_cb(self, msg: Odometry):
        now = time.time()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self.odom_writer.writerow([
            f"{now:.4f}",
            f"{p.x:.5f}", f"{p.y:.5f}", f"{p.z:.5f}",
            f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}", f"{q.w:.6f}",
            f"{v.x:.5f}", f"{v.y:.5f}", f"{v.z:.5f}", f"{w.z:.5f}",
        ])

    def lidar_cb(self, msg: PointCloud2):
        self.frame_idx += 1
        # --every 프레임마다 한 번씩만 저장
        if (self.frame_idx % self.every) != 0:
            return

        now = time.time()
        rows = []
        try:
            pts = point_cloud2.read_points(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
            )
            for pt in pts:
                rows.append([
                    self.saved_frame, f"{now:.4f}",
                    f"{float(pt[0]):.4f}", f"{float(pt[1]):.4f}",
                    f"{float(pt[2]):.4f}", f"{float(pt[3]):.3f}",
                ])
        except Exception:
            # intensity 필드가 없으면 xyz 만 저장
            pts = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
            for pt in pts:
                rows.append([
                    self.saved_frame, f"{now:.4f}",
                    f"{float(pt[0]):.4f}", f"{float(pt[1]):.4f}",
                    f"{float(pt[2]):.4f}", "0",
                ])

        self.lidar_writer.writerows(rows)
        self.saved_frame += 1
        if self.saved_frame % 10 == 0:
            self.get_logger().info(
                f"라이다 {self.saved_frame}프레임 저장됨 (마지막 {len(rows)}점)"
            )

    def destroy_node(self):
        try:
            self.odom_file.close()
            self.lidar_file.close()
            self.get_logger().info("CSV 파일을 저장하고 닫았습니다.")
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="Go2 라이다+오도메트리 CSV 로거")
    parser.add_argument("--outdir", default=os.path.expanduser("~"),
                        help="CSV 저장 폴더 (기본: 홈 디렉터리)")
    parser.add_argument("--every", type=int, default=1,
                        help="라이다를 몇 프레임마다 저장할지 (기본 1=모두)")
    args = parser.parse_args()

    rclpy.init()
    node = Go2CsvLogger(outdir=args.outdir, every=args.every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
