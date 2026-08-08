#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
repro_monitor.py — 재현성 실험용 관측 노드

하는 일 (Point-LIO 에는 손대지 않음):
  1) /aft_mapped_to_init (nav_msgs/Odometry) 개수를 센다
     → Point-LIO 가 "실제로 처리한 스캔 수". bag 의 6,620 과 비교하면 드롭 여부가 나온다.
  2) 궤적을 CSV 로 저장하고 경로길이 / 시작-끝 거리를 계산한다
     → 폐곡선 주행이므로 시작-끝 거리가 곧 누적 드리프트다.
  3) CPU 사용률을 1초마다 샘플링한다
     → 실행마다 부하가 달랐는지 확인용.

주의: 점군 토픽은 일부러 구독하지 않는다. 역직렬화 비용이 실험 자체를 방해한다.

사용: python3 repro_monitor.py <출력디렉터리>
종료: SIGINT(Ctrl+C) 또는 SIGTERM → summary.json / traj.csv 저장
"""

import json
import math
import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry

ODOM_TOPIC = '/aft_mapped_to_init'


def cpu_snapshot():
    """/proc/stat 첫 줄에서 (idle, total) 누적 tick 을 읽는다."""
    with open('/proc/stat') as f:
        parts = f.readline().split()
    vals = [int(v) for v in parts[1:]]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)   # idle + iowait
    return idle, sum(vals)


class ReproMonitor(Node):
    def __init__(self, outdir):
        super().__init__('repro_monitor')
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)

        self.count = 0
        self.traj = []          # (stamp_sec, x, y, z, wall_t)
        self.first_wall = None
        self.last_wall = None
        self.cpu = []           # (wall_t, usage_percent)
        self._lock = threading.Lock()

        # Point-LIO 는 기본 QoS(RELIABLE)로 발행한다. RELIABLE 로 받아야 셈이 정확하다.
        # (BEST_EFFORT 로 받으면 우리 쪽에서 흘려버려 드롭 판정이 오염된다)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )
        self.sub = self.create_subscription(Odometry, ODOM_TOPIC, self.cb, qos)

        self._stop = threading.Event()
        self._cpu_thread = threading.Thread(target=self.cpu_loop, daemon=True)
        self._cpu_thread.start()

        self.get_logger().info(f'감시 시작: {ODOM_TOPIC} → {outdir}')

    def cb(self, msg):
        now = time.time()
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self.count += 1
            if self.first_wall is None:
                self.first_wall = now
            self.last_wall = now
            q = msg.pose.pose.orientation
            self.traj.append((stamp, p.x, p.y, p.z, now, q.x, q.y, q.z, q.w))
            if self.count % 500 == 0:
                self.get_logger().info(f'  처리 프레임 {self.count}')

    def cpu_loop(self):
        prev = cpu_snapshot()
        while not self._stop.is_set():
            time.sleep(1.0)
            cur = cpu_snapshot()
            d_idle = cur[0] - prev[0]
            d_tot = cur[1] - prev[1]
            prev = cur
            if d_tot > 0:
                usage = 100.0 * (1.0 - d_idle / d_tot)
                with self._lock:
                    self.cpu.append((time.time(), round(usage, 1)))

    def save(self):
        self._stop.set()
        with self._lock:
            traj = list(self.traj)
            cpu = list(self.cpu)
            count = self.count

        # 궤적 CSV
        with open(os.path.join(self.outdir, 'traj.csv'), 'w') as f:
            f.write('stamp,x,y,z,wall,qx,qy,qz,qw\n')
            for row in traj:
                f.write('%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n' % row)

        # CPU CSV
        with open(os.path.join(self.outdir, 'cpu.csv'), 'w') as f:
            f.write('wall,usage_percent\n')
            for t, u in cpu:
                f.write('%.3f,%.1f\n' % (t, u))

        # 경로길이 / 시작-끝 거리
        path_len = 0.0
        for i in range(1, len(traj)):
            dx = traj[i][1] - traj[i - 1][1]
            dy = traj[i][2] - traj[i - 1][2]
            dz = traj[i][3] - traj[i - 1][3]
            path_len += math.sqrt(dx * dx + dy * dy + dz * dz)

        start_end = None
        if len(traj) >= 2:
            dx = traj[-1][1] - traj[0][1]
            dy = traj[-1][2] - traj[0][2]
            dz = traj[-1][3] - traj[0][3]
            start_end = math.sqrt(dx * dx + dy * dy + dz * dz)
            start_end_xy = math.sqrt(dx * dx + dy * dy)
        else:
            start_end_xy = None

        # 처리 공백(스톨) 탐지 — 연속 프레임 간 실시간 간격이 1초 넘으면 기록
        stalls = []
        for i in range(1, len(traj)):
            gap = traj[i][4] - traj[i - 1][4]
            if gap > 1.0:
                stalls.append({'index': i, 'gap_s': round(gap, 2)})

        cpu_vals = [u for _, u in cpu]
        summary = {
            'odom_frames': count,
            'traj_points': len(traj),
            'path_length_m': round(path_len, 3),
            'start_end_dist_m': None if start_end is None else round(start_end, 3),
            'start_end_dist_xy_m': None if start_end_xy is None else round(start_end_xy, 3),
            'wall_span_s': None if (self.first_wall is None or self.last_wall is None)
                           else round(self.last_wall - self.first_wall, 1),
            'cpu_mean_percent': round(sum(cpu_vals) / len(cpu_vals), 1) if cpu_vals else None,
            'cpu_max_percent': max(cpu_vals) if cpu_vals else None,
            'cpu_samples': len(cpu_vals),
            'stall_count_gt_1s': len(stalls),
            'stalls': stalls[:20],
        }
        with open(os.path.join(self.outdir, 'monitor.json'), 'w') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        print('\n[repro_monitor] 저장 완료')
        print(json.dumps(summary, indent=2, ensure_ascii=False))


def main():
    if len(sys.argv) < 2:
        print('사용: python3 repro_monitor.py <출력디렉터리>')
        sys.exit(1)
    outdir = sys.argv[1]

    rclpy.init()
    node = ReproMonitor(outdir)

    stop = threading.Event()

    def handler(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
