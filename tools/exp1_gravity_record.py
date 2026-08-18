#!/usr/bin/env python3
"""
exp1_gravity_record.py -- 실험 1: 정지 자세 중력 방향 기록

목적
----
로봇을 완전 정지(엎드림)시킨 상태에서, 네 개의 IMU 소스가 각각
"아래(중력)"를 어디로 보는지 3분간 기록한다.

기록 소스 (source 라벨)
-----------------------
  L1       : /utlidar/imu        (L1 원본, 자이로+가속도 모두 L1)
  body     : /lowstate           (몸통 IMU 원본, 500Hz)
  body_lf  : /lf/lowstate         (몸통 IMU 저주파, 20Hz)
  fix      : /l1_imu_fixed        (L1 자이로 + 몸통가속도를 R_LB 로 회전한 합성)
             * l1_imu_fix.py 가 켜져 있어야 나온다.

파생값
------
  mag              = sqrt(ax^2+ay^2+az^2)   -> 9.81 근처여야 정상
  gravity_angle_deg= 측정 가속도 벡터와 참값 (0,0,-9.81) 사이 각도(도)
                     정지 시 IMU 가속도는 -중력(위 방향 반력)이므로
                     측정벡터와 (0,0,+9.81) 사이 각을 재는 것이 물리적으로 맞다.
                     여기서는 "측정벡터 방향이 수직(+z)에서 얼마나 벗어났나"를 본다.

CSV
---
실행 즉시 파일을 열고 매 샘플을 한 줄씩 바로 기록한다 (flush).
컬럼: t, source, ax, ay, az, gx, gy, gz, mag, gravity_angle_deg

사용
    python3 exp1_gravity_record.py [출력.csv] [기록초]
    기본: exp1_gravity_YYYYmmdd_HHMMSS.csv , 180초
"""
import csv
import math
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState

DURATION_DEFAULT = 180.0  # 3분


def accel_metrics(ax, ay, az):
    """중력 크기와, +z(수직 위)에서 벗어난 각도(도)."""
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if mag < 1e-6:
        return mag, float('nan')
    # 측정 가속도 벡터가 +z 축(수직 위, 정지 시 반력 방향)에서 벗어난 각
    cos_a = az / mag
    cos_a = max(-1.0, min(1.0, cos_a))
    angle = math.degrees(math.acos(cos_a))
    return mag, angle


class GravityRecorder(Node):
    def __init__(self, csv_path, duration):
        super().__init__('exp1_gravity_record')
        self.duration = duration
        self.t0 = time.time()
        self.counts = {'L1': 0, 'body': 0, 'body_lf': 0, 'fix': 0}

        # CSV: 실행 즉시 열고 헤더 기록
        self.f = open(csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'source', 'ax', 'ay', 'az',
                         'gx', 'gy', 'gz', 'mag', 'gravity_angle_deg'])
        self.f.flush()
        self.csv_path = csv_path

        # 구독 (4 소스)
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
            LowState, '/lf/lowstate',
            lambda m: self.on_lowstate(m, 'body_lf'), qos_profile_sensor_data)

        self.get_logger().info('기록 시작 -> %s  (%.0f초)' % (csv_path, duration))
        self.get_logger().info('로봇을 정지(엎드림) 상태로 유지할 것.')
        self.timer = self.create_timer(5.0, self.tick)

    def _write(self, source, ax, ay, az, gx, gy, gz):
        mag, angle = accel_metrics(ax, ay, az)
        t = time.time() - self.t0
        self.w.writerow(['%.4f' % t, source,
                         '%.6f' % ax, '%.6f' % ay, '%.6f' % az,
                         '%.6f' % gx, '%.6f' % gy, '%.6f' % gz,
                         '%.5f' % mag, '%.4f' % angle])
        self.f.flush()  # 매 샘플 즉시 디스크로
        self.counts[source] += 1

    def on_imu(self, msg, source):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self._write(source, a.x, a.y, a.z, g.x, g.y, g.z)

    def on_lowstate(self, msg, source):
        a = msg.imu_state.accelerometer
        g = msg.imu_state.gyroscope
        self._write(source, a[0], a[1], a[2], g[0], g[1], g[2])

    def tick(self):
        el = time.time() - self.t0
        c = self.counts
        self.get_logger().info(
            '%.0f/%.0fs  수신 L1=%d body=%d body_lf=%d fix=%d'
            % (el, self.duration, c['L1'], c['body'], c['body_lf'], c['fix']))
        if el >= self.duration:
            self.finish()

    def finish(self):
        self.f.flush()
        self.f.close()
        self.get_logger().info('=== 기록 완료 ===')
        self.get_logger().info('저장: %s' % self.csv_path)
        c = self.counts
        self.get_logger().info(
            '총 샘플: L1=%d body=%d body_lf=%d fix=%d'
            % (c['L1'], c['body'], c['body_lf'], c['fix']))
        for s in ('L1', 'body', 'body_lf', 'fix'):
            if c[s] == 0:
                self.get_logger().warn(
                    '  %s 수신 0 -> 토픽이 안 나온다. (fix면 l1_imu_fix.py 켰는지 확인)' % s)
        raise SystemExit


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else \
        'exp1_gravity_%s.csv' % datetime.now().strftime('%Y%m%d_%H%M%S')
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else DURATION_DEFAULT

    rclpy.init()
    node = GravityRecorder(csv_path, duration)
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


if __name__ == '__main__':
    main()
