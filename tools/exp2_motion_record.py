#!/usr/bin/env python3
"""
exp2_motion_record.py -- 실험 2: 움직임 중 IMU + 오도메트리 기록 (상관계수용)

목적
----
로봇을 조종기로 직진/회전시키며 세 IMU(L1, body, fix)와 오도메트리를 함께
기록한다. 후처리에서 "IMU 가속도 vs odom 속도의 1차 미분(=실제 가속도)"의
상관계수(동조)와 RMSE(정확도)를 낸다.

동작 시퀀스 (조종기로 직접)
---------------------------
  1. 정지 15초    (초기화)
  2. 직진 2m
  3. 정지 20초
  4. 제자리 왼쪽 2회
  5. 정지 20초
  6. 제자리 오른쪽 2회
  7. 정지 20초

기록 소스
---------
  L1   : /utlidar/imu       ax,ay,az, gx,gy,gz
  body : /lowstate          ax,ay,az, gx,gy,gz
  fix  : /l1_imu_fixed      ax,ay,az, gx,gy,gz
  odom : /utlidar/robot_odom  pos(xyz), quat(xyzw), vel(xyz), angvel(xyz)
         * fix 를 기록하려면 l1_imu_fix.py 가 켜져 있어야 한다.

저장 방식
---------
raw 값을 전부 저장한다. 프레임 정렬 / 중력 제거 / 미분 / 상관은 후처리.
컬럼 수가 소스마다 다르므로, 공통 스키마에 해당 없는 칸은 비운다.

CSV 컬럼
--------
t, source,
ax, ay, az, gx, gy, gz,               # IMU (odom 행에서는 빈칸)
px, py, pz, qx, qy, qz, qw,           # odom pose (IMU 행에서는 빈칸)
vx, vy, vz, wx, wy, wz                # odom twist (IMU 행에서는 빈칸)

사용
    python3 exp2_motion_record.py [출력.csv] [기록초]
    기본: exp2_motion_YYYYmmdd_HHMMSS.csv , 180초
    동작을 다 마치면 Ctrl+C 로 끝내도 된다 (그때까지 저장됨).
"""
import csv
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from unitree_go.msg import LowState

DURATION_DEFAULT = 180.0

HEADER = ['t', 'source',
          'ax', 'ay', 'az', 'gx', 'gy', 'gz',
          'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw',
          'vx', 'vy', 'vz', 'wx', 'wy', 'wz']


class MotionRecorder(Node):
    def __init__(self, csv_path, duration):
        super().__init__('exp2_motion_record')
        self.duration = duration
        self.t0 = time.time()
        self.counts = {'L1': 0, 'body': 0, 'fix': 0, 'odom': 0}

        self.f = open(csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(HEADER)
        self.f.flush()
        self.csv_path = csv_path

        self.create_subscription(
            Imu, '/utlidar/imu',
            lambda m: self.on_imu(m, 'L1'), qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/l1_imu_fixed',
            lambda m: self.on_imu(m, 'fix'), qos_profile_sensor_data)
        self.create_subscription(
            LowState, '/lowstate',
            lambda m: self.on_lowstate(m, 'body'), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/utlidar/robot_odom',
            self.on_odom, qos_profile_sensor_data)

        self.get_logger().info('기록 시작 -> %s  (최대 %.0f초)' % (csv_path, duration))
        self.get_logger().info('시퀀스: 정지15s -> 직진2m -> 정지20s -> 좌2회 -> 정지20s -> 우2회 -> 정지20s')
        self.get_logger().info('동작 끝나면 Ctrl+C 로 종료해도 됨.')
        self.timer = self.create_timer(5.0, self.tick)

    def _row_imu(self, source, a, g):
        t = time.time() - self.t0
        self.w.writerow(['%.4f' % t, source,
                         '%.6f' % a[0], '%.6f' % a[1], '%.6f' % a[2],
                         '%.6f' % g[0], '%.6f' % g[1], '%.6f' % g[2],
                         '', '', '', '', '', '', '',
                         '', '', '', '', '', ''])
        self.f.flush()
        self.counts[source] += 1

    def on_imu(self, msg, source):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self._row_imu(source, (a.x, a.y, a.z), (g.x, g.y, g.z))

    def on_lowstate(self, msg, source):
        a = msg.imu_state.accelerometer
        g = msg.imu_state.gyroscope
        self._row_imu(source, (a[0], a[1], a[2]), (g[0], g[1], g[2]))

    def on_odom(self, msg):
        t = time.time() - self.t0
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self.w.writerow(['%.4f' % t, 'odom',
                         '', '', '', '', '', '',
                         '%.6f' % p.x, '%.6f' % p.y, '%.6f' % p.z,
                         '%.6f' % q.x, '%.6f' % q.y, '%.6f' % q.z, '%.6f' % q.w,
                         '%.6f' % v.x, '%.6f' % v.y, '%.6f' % v.z,
                         '%.6f' % w.x, '%.6f' % w.y, '%.6f' % w.z])
        self.f.flush()
        self.counts['odom'] += 1

    def tick(self):
        el = time.time() - self.t0
        c = self.counts
        self.get_logger().info(
            '%.0f/%.0fs  L1=%d body=%d fix=%d odom=%d'
            % (el, self.duration, c['L1'], c['body'], c['fix'], c['odom']))
        if el >= self.duration:
            self.finish()

    def finish(self):
        self.f.flush()
        self.f.close()
        self.get_logger().info('=== 기록 완료 ===')
        self.get_logger().info('저장: %s' % self.csv_path)
        c = self.counts
        self.get_logger().info('총 샘플: L1=%d body=%d fix=%d odom=%d'
                               % (c['L1'], c['body'], c['fix'], c['odom']))
        for s in ('L1', 'body', 'fix', 'odom'):
            if c[s] == 0:
                self.get_logger().warn('  %s 수신 0 -> 토픽 확인 (fix면 l1_imu_fix.py)' % s)
        raise SystemExit


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else \
        'exp2_motion_%s.csv' % datetime.now().strftime('%Y%m%d_%H%M%S')
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else DURATION_DEFAULT

    rclpy.init()
    node = MotionRecorder(csv_path, duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.f.flush()
            node.f.close()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    print('저장 완료:', csv_path)


if __name__ == '__main__':
    main()
