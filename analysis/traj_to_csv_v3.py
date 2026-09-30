#!/usr/bin/env python3
"""
traj_to_csv_v3.py
-----------------
nav_msgs/Odometry 궤적 bag 을 CSV 로 변환한다.
v2 와 출력 CSV 형식은 완전히 동일하고, 요약에 정지 드리프트 지표를 추가했다.

CSV 컬럼 (v2 와 같음):
  t_sec, x, y, z, qx, qy, qz, qw, yaw_deg, dist_from_origin

추가된 요약 (정지 구간 테스트 T1/T2 판정용):
  - yaw 선형회귀 기울기 [deg/s]  : 정지 중 회전 드리프트율
  - 회귀 잔차 RMS [deg]          : 작을수록 '등속 회전'
  - 가짜 이동 속도 [cm/s]        : 정지인데 움직인다고 추정한 속도
  - 등가반지름 [m]               : v / |w|
  - roll / pitch 변동폭          : 중력으로 잡히는 축이 안정적인지

주의 — 재생 배속
  bag 을 -r 0.5 로 재생하며 녹화하면 CSV 의 t_sec 은 실제(bag) 시간의 2배다.
  --rate 로 배속을 알려주면 bag 시간 기준으로 환산해 준다. (기본 0.5)

사용법:
  unset CYCLONEDDS_URI
  export ROS_DOMAIN_ID=99
  source /opt/ros/humble/setup.bash

  python3 traj_to_csv_v3.py ~/data/bags/still_T2 still_T2.csv
  python3 traj_to_csv_v3.py ~/data/bags/still_T2/still_T2_0.db3 still_T2.csv
  python3 traj_to_csv_v3.py <bag> <out.csv> --topic /aft_mapped_to_init --rate 0.5
"""

import argparse
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DEFAULT_TOPIC = "/aft_mapped_to_init"
DIVERGE_M = 100.0          # 원점거리가 이 값을 넘으면 발산으로 본다
R2D = 180.0 / math.pi


def resolve_bag(path):
    if os.path.isdir(path):
        for f in sorted(os.listdir(path)):
            if f.endswith(".db3"):
                return os.path.join(path, f)
        sys.exit(f"[에러] {path} 안에 .db3 가 없습니다")
    return path


def quat_to_rpy(qx, qy, qz, qw):
    """ZYX 오일러각 [deg]. roll(x) -> pitch(y) -> yaw(z)"""
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    s = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll * R2D, pitch * R2D, yaw * R2D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--rate", type=float, default=0.5,
                    help="녹화 당시 재생 배속. bag 시간 환산용 (기본 0.5)")
    args = ap.parse_args()

    db3 = resolve_bag(args.bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=db3, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in types:
        sys.exit(f"[에러] bag 에 {args.topic} 가 없습니다.\n"
                 f"       있는 토픽: {list(types)}")
    MsgType = get_message(types[args.topic])

    rows = []
    t0 = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != args.topic:
            continue
        m = deserialize_message(data, MsgType)
        t = t_ns * 1e-9
        if t0 is None:
            t0 = t
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        rows.append((t - t0, p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                     yaw, math.sqrt(p.x**2 + p.y**2 + p.z**2)))

    if not rows:
        sys.exit(f"[에러] {args.topic} 메시지가 0개입니다")

    with open(args.out, "w") as f:
        f.write("t_sec,x,y,z,qx,qy,qz,qw,yaw_deg,dist_from_origin\n")
        for r in rows:
            f.write("%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f\n" % r)

    # ── 배열화 ────────────────────────────────────────────
    a = np.array(rows)
    t = a[:, 0]
    # 0.5배속 재생 중 녹화하면 벽시계 시간이 bag 시간의 2배다.
    # 따라서 bag 시간 = 벽시계 시간 x 배속.
    tb = t * args.rate if args.rate > 0 else t      # bag 시간
    x, y, z = a[:, 1], a[:, 2], a[:, 3]
    dist = a[:, 9]
    yaw_u = np.rad2deg(np.unwrap(np.deg2rad(a[:, 8])))

    rp = np.array([quat_to_rpy(*a[i, 4:8]) for i in range(len(a))])
    roll, pitch = rp[:, 0], rp[:, 1]

    xr, yr = x - x[0], y - y[0]
    path = float(np.sum(np.hypot(np.diff(xr), np.diff(yr))))
    loop = float(math.hypot(xr[-1], yr[-1]))
    dur_b = float(tb[-1] - tb[0])

    div_idx = np.where(dist > DIVERGE_M)[0]

    # ── v2 와 동일한 요약 ─────────────────────────────────
    print(f"완료: {args.out}   (토픽: {args.topic})")
    print(f"  메시지 수    : {len(rows)}")
    print(f"  기록 시간    : {t[-1]:.1f} s   (배속 {args.rate} -> bag {dur_b:.1f} s)")
    print(f"  원점 최대거리: {dist.max():.1f} m")
    if len(div_idx):
        print(f"  ** 발산 감지 t={t[div_idx[0]]:.4f} s "
              f"(원점거리 {DIVERGE_M:.0f}m 초과) **")
    else:
        print(f"  발산 없음(원점거리 {DIVERGE_M:.0f}m 이내 유지) -> 정상 궤적일 가능성")
    print(f"  시작점: ({x[0]:.2f}, {y[0]:.2f})")
    print(f"  끝점  : ({x[-1]:.2f}, {y[-1]:.2f})")
    print(f"  폐루프 오차(시작-끝 xy거리): {loop:.3f} m")

    # ── 추가: 정지 드리프트 지표 ──────────────────────────
    if dur_b <= 0 or len(a) < 10:
        return
    print()
    print("  ── 드리프트 지표 (bag 시간 기준) ──")
    coef = np.polyfit(tb, yaw_u, 1)
    res = float((yaw_u - np.polyval(coef, tb)).std())
    w = float(coef[0])                 # deg/s
    v = path / dur_b                   # m/s
    print(f"  총 이동경로   : {path:.2f} m   ({v*100:.0f} cm/s)")
    if path > 0:
        print(f"  드리프트율    : {100*loop/path:.2f} %")
    print(f"  yaw 총변화    : {yaw_u[-1]-yaw_u[0]:+.1f} deg")
    print(f"  yaw 회귀      : {w:+.2f} deg/s   (잔차 RMS {res:.2f} deg)")
    if abs(w) > 1e-6:
        print(f"  등가반지름    : {v/abs(math.radians(w)):.2f} m")
    print(f"  roll  변동폭  : {roll.max()-roll.min():.1f} deg "
          f"({roll.min():.1f} ~ {roll.max():.1f})")
    print(f"  pitch 변동폭  : {pitch.max()-pitch.min():.1f} deg "
          f"({pitch.min():.1f} ~ {pitch.max():.1f})")
    print(f"  z 변화        : {z[-1]-z[0]:+.3f} m")

    # 판정 (정지 구간 테스트용)
    print()
    if len(div_idx):
        print("  -> 발산했으므로 위 회귀 지표는 무의미하다. 발산 시점만 볼 것.")
        print(f"     발산까지 bag {tb[div_idx[0]]:.1f} s, 그때 원점거리 "
              f"{dist[div_idx[0]]:.1f} m")
    elif abs(w) > 5.0:
        print(f"  -> 정지 드리프트 재현됨 ({w:+.2f} deg/s). 기준 -12.7 deg/s 대비 "
              f"{100*abs(w)/12.7:.0f} %")
    elif abs(w) > 1.0:
        print(f"  -> 드리프트가 크게 줄었다 ({w:+.2f} deg/s). 바꾼 변수가 원인에 가깝다.")
    else:
        print(f"  -> 드리프트 거의 없음 ({w:+.2f} deg/s). 바꾼 변수가 원인이었다.")


if __name__ == "__main__":
    main()
