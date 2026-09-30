#!/usr/bin/env python3
"""
imu_mean.py
-----------
bag 안의 sensor_msgs/Imu 토픽 하나를 읽어, 자이로·가속도의 평균과
적분 각도를 출력한다.

목적:
  gyro_bias_check.py 는 /utlidar/imu (L1 원본) 만 봤다.
  Point-LIO 가 실제로 먹는 것은 /l1_imu_fixed (fix 노드 출력) 이므로,
  그쪽이 오염됐는지 따로 확인해야 한다.

  판정: |자이로 z 평균| 이 0.22 rad/s 근처  -> fix 노드가 원인
        0 근처                              -> fix 노드는 무죄, Point-LIO 내부 문제

사용법:
  unset CYCLONEDDS_URI
  export ROS_DOMAIN_ID=99
  source /opt/ros/humble/setup.bash

  python3 imu_mean.py ~/data/bags/still_T1b
  python3 imu_mean.py ~/data/bags/still_T1b --topic /l1_imu_fixed

  # 원본과 나란히 비교하고 싶을 때
  python3 imu_mean.py ~/data/bags/lio_test_bag_loop_run2 \
      --topic /utlidar/imu --t0 239.8 --t1 264.8

옵션:
  --topic   토픽 이름 (생략하면 bag 안의 sensor_msgs/Imu 토픽을 전부 처리)
  --t0 --t1 bag 시작 기준 구간 [s] (생략하면 전체)
"""

import argparse
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

R2D = 180.0 / math.pi
IMU_TYPE = "sensor_msgs/msg/Imu"


def resolve_bag(path):
    if os.path.isdir(path):
        for f in sorted(os.listdir(path)):
            if f.endswith(".db3"):
                return os.path.join(path, f)
        sys.exit(f"[에러] {path} 안에 .db3 가 없습니다")
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default=None)
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
    args = ap.parse_args()

    db3 = resolve_bag(args.bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=db3, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if args.topic:
        if args.topic not in types:
            sys.exit(f"[에러] bag 에 {args.topic} 가 없습니다.\n"
                     f"       있는 토픽: {list(types)}")
        targets = [args.topic]
    else:
        targets = [n for n, ty in types.items() if ty == IMU_TYPE]
        if not targets:
            sys.exit(f"[에러] sensor_msgs/Imu 토픽이 없습니다. 있는 토픽: {list(types)}")

    cls = {n: get_message(types[n]) for n in targets}
    buf = {n: [] for n in targets}

    t0_bag = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if t0_bag is None:
            t0_bag = t
        if topic not in buf:
            continue
        buf[topic].append((t - t0_bag, deserialize_message(data, cls[topic])))

    print(f"bag : {args.bag}")
    if args.t0 is not None or args.t1 is not None:
        print(f"구간: {args.t0} ~ {args.t1} s")
    print("=" * 60)

    for name in targets:
        rows = buf[name]
        if args.t0 is not None:
            rows = [r for r in rows if r[0] >= args.t0]
        if args.t1 is not None:
            rows = [r for r in rows if r[0] <= args.t1]
        if len(rows) < 10:
            print(f"[{name}] 샘플 부족 ({len(rows)}개)\n")
            continue

        t = np.array([r[0] for r in rows])
        w = np.array([[m.angular_velocity.x, m.angular_velocity.y,
                       m.angular_velocity.z] for _, m in rows])
        a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                       m.linear_acceleration.z] for _, m in rows])

        dur = t[-1] - t[0]
        wm, ws = w.mean(axis=0), w.std(axis=0)
        am = a.mean(axis=0)
        integ = (np.trapezoid(w, t, axis=0) if hasattr(np, "trapezoid")
                 else np.trapz(w, t, axis=0)) * R2D

        # 프레임 확인용: 중력이 어느 축을 향하는가, 기울기는 몇 도인가
        g = np.linalg.norm(am)
        ax = int(np.argmax(np.abs(am)))
        tilt = math.degrees(math.acos(min(1.0, abs(am[ax]) / g))) if g > 0 else 0.0

        print(f"[{name}]  ({types[name]})")
        print(f"  샘플 {len(rows)}개, {dur:.1f}s, {len(rows)/dur:.0f} Hz")
        print(f"  자이로 평균 [rad/s]  x {wm[0]:+.5f}  y {wm[1]:+.5f}  z {wm[2]:+.5f}")
        print(f"  자이로 평균 [deg/s]  x {wm[0]*R2D:+.3f}  y {wm[1]*R2D:+.3f}  z {wm[2]*R2D:+.3f}")
        print(f"  표준편차   [deg/s]  x {ws[0]*R2D:.3f}  y {ws[1]*R2D:.3f}  z {ws[2]*R2D:.3f}")
        print(f"  적분 각도  [deg]    x {integ[0]:+.1f}  y {integ[1]:+.1f}  z {integ[2]:+.1f}")
        print(f"  가속도 평균[m/s^2]  x {am[0]:+.3f}  y {am[1]:+.3f}  z {am[2]:+.3f}")
        print(f"    크기 {g:.3f} (중력 대비 {100*g/9.807:.1f} %), "
              f"최대축 {'+' if am[ax] > 0 else '-'}{'xyz'[ax]}, 그 축에서 {tilt:.1f} deg 기울음")

        z = abs(wm[2])
        print(f"  -> |자이로 z| = {z:.5f} rad/s ({z*R2D:.2f} deg/s), "
              f"기준 0.221 대비 {100*z/0.221:.1f} %")
        if z > 0.11:
            print("     ** 이 토픽의 자이로가 오염됐습니다. 여기가 원인입니다. **")
        elif z > 0.02:
            print("     ** 일부 오염. 전체는 설명 못 하지만 무시할 수준도 아닙니다. **")
        else:
            print("     자이로 정상. 이 토픽은 무죄입니다.")
        print()


if __name__ == "__main__":
    main()
