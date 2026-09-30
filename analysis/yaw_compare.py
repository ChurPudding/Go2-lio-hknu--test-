#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yaw_compare.py

ekf_test1 재생으로 만든 "출력 bag"에서 두 yaw 소스를 비교한다.
  - Point-LIO yaw : /aft_mapped_to_init   (nav_msgs/Odometry)
  - 다리 오도메트리 yaw : /utlidar/robot_odom (nav_msgs/Odometry)

하는 일
  1) 두 토픽에서 (시각, yaw) 시계열을 뽑는다
  2) 다리 속도로 정지 구간을 자동 검출하고, 그 사이 이동 구간을 폐루프 A/B로 분할한다
  3) 다리 yaw를 Point-LIO 시각축에 보간해 시각 정렬한다
  4) 두 yaw(누적)와 그 차이를 그리고, -170도 방향 도달 지점을 표시한다

판정 요령
  - 두 yaw가 "함께" -170도로 벌어짐  → 입력/기하(degeneracy) 문제
  - Point-LIO만 벌어짐               → Point-LIO 처리(초기화/필터/extrinsic) 문제

사용
  python3 yaw_compare.py ~/data/bags/plout_ekf1_1530
  python3 yaw_compare.py <bag> --still-speed 0.05 --min-still 3.0 --target -170
"""

import sys
import math
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

PL_TOPIC = "/aft_mapped_to_init"     # Point-LIO 출력
LEG_TOPIC = "/utlidar/robot_odom"    # 다리 오도메트리 (기준선)


def yaw_from_quat(q):
    """쿼터니언 -> yaw(z축 회전).
    yaw = atan2( 2(w*z + x*y),  1 - 2(y^2 + z^2) )
    """
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def stamp_sec(msg):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


def read_bag(path):
    """출력 bag에서 PL/다리 yaw 시계열을 읽는다.
    시각축은 header.stamp 를 쓴다 (Point-LIO는 입력 LiDAR 스탬프를 그대로 실어
    발행하고, 다리 오도메트리도 원본 스탬프를 유지하므로 둘 다 bag 원본 시간축)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in (PL_TOPIC, LEG_TOPIC):
        if need not in types:
            sys.exit(f"[에러] bag에 {need} 가 없습니다. 담긴 토픽: {list(types)}")

    MsgPL = get_message(types[PL_TOPIC])
    MsgLeg = get_message(types[LEG_TOPIC])

    pl = []   # (t, yaw)
    leg = []  # (t, x, y, yaw)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == PL_TOPIC:
            m = deserialize_message(data, MsgPL)
            pl.append((stamp_sec(m), yaw_from_quat(m.pose.pose.orientation)))
        elif topic == LEG_TOPIC:
            m = deserialize_message(data, MsgLeg)
            p = m.pose.pose.position
            leg.append((stamp_sec(m), p.x, p.y,
                        yaw_from_quat(m.pose.pose.orientation)))

    if not pl:
        sys.exit(f"[에러] {PL_TOPIC} 메시지가 0개입니다. Point-LIO가 발행됐는지 확인하세요.")
    if not leg:
        sys.exit(f"[에러] {LEG_TOPIC} 메시지가 0개입니다.")
    return np.array(pl), np.array(leg)


def find_still_segments(t, still_mask, min_dur):
    """정지 마스크에서 min_dur 이상 지속된 구간만 (시작, 끝) 리스트로 반환."""
    segs = []
    i, n = 0, len(t)
    while i < n:
        if still_mask[i]:
            j = i
            while j + 1 < n and still_mask[j + 1]:
                j += 1
            if t[j] - t[i] >= min_dur:
                segs.append((t[i], t[j]))
            i = j + 1
        else:
            i += 1
    return segs


def loops_from_still(still_segs):
    """정지 구간 사이의 이동 구간을 폐루프로 본다.
    still-loopA-still-loopB-still 이면 정지 3개 -> 루프 2개(A,B)."""
    loops = []
    prev_end = None
    for (s, e) in still_segs:
        if prev_end is not None and s > prev_end:
            loops.append((prev_end, s))
        prev_end = e
    return loops


def net_yaw(t_arr, yaw_deg, t0, t1):
    """t0~t1 구간의 순(net) yaw 변화량[deg]. 폐루프면 이상적으로 0에 가까움."""
    y0 = np.interp(t0, t_arr, yaw_deg)
    y1 = np.interp(t1, t_arr, yaw_deg)
    return y1 - y0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="출력 bag 경로 (예: ~/data/bags/plout_ekf1_1530)")
    ap.add_argument("--still-speed", type=float, default=0.05,
                    help="정지 판정 속도 임계값 [m/s] (기본 0.05, ZUPT와 동일)")
    ap.add_argument("--min-still", type=float, default=3.0,
                    help="A/B 경계로 인정할 최소 정지 지속시간 [s] (기본 3.0)")
    ap.add_argument("--target", type=float, default=-170.0,
                    help="관심 드리프트 각도 [deg] (기본 -170)")
    ap.add_argument("--out", default="yaw_compare.png", help="그림 저장 경로")
    args = ap.parse_args()

    pl, leg = read_bag(args.bag)

    # ── 시간축을 공통 원점(가장 이른 스탬프)으로 이동
    t_ref = min(pl[0, 0], leg[0, 0])
    pl_t = pl[:, 0] - t_ref
    leg_t = leg[:, 0] - t_ref

    # ── yaw unwrap (±180 경계에서 튀지 않도록) 후 절대 누적각[deg]
    pl_yaw = np.degrees(np.unwrap(pl[:, 1]))
    leg_yaw = np.degrees(np.unwrap(leg[:, 3]))

    # ── 다리 속도(위치 미분)로 정지 구간 검출
    lx, ly = leg[:, 1], leg[:, 2]
    dt = np.gradient(leg_t)
    speed = np.hypot(np.gradient(lx), np.gradient(ly)) / np.clip(dt, 1e-3, None)
    still_mask = speed < args.still_speed
    still_segs = find_still_segments(leg_t, still_mask, args.min_still)
    loops = loops_from_still(still_segs)

    # ── 다리 yaw를 PL 시각축에 보간(시각 정렬)
    leg_yaw_on_pl = np.interp(pl_t, leg_t, leg_yaw)

    # ── 시작을 0으로 맞춘 누적 yaw, 그리고 두 소스의 차(=Point-LIO 고유 편차)
    pl_acc = pl_yaw - pl_yaw[0]
    leg_acc = leg_yaw_on_pl - leg_yaw_on_pl[0]
    diverg = pl_acc - leg_acc

    # ── 콘솔 요약
    print("=" * 60)
    print(f"bag                : {args.bag}")
    print(f"PL 샘플 / 다리 샘플 : {len(pl_t)} / {len(leg_t)}")
    print(f"정지 구간          : {len(still_segs)}개  "
          f"(임계 {args.still_speed} m/s, 최소 {args.min_still} s)")
    for k, (s, e) in enumerate(still_segs, 1):
        print(f"   정지#{k}: {s:6.1f} ~ {e:6.1f} s  (지속 {e - s:4.1f} s)")
    print(f"폐루프            : {len(loops)}개")
    for k, (s, e) in enumerate(loops, 1):
        pl_net = net_yaw(pl_t, pl_yaw, s, e)
        leg_net = net_yaw(pl_t, leg_yaw_on_pl, s, e)
        tag = "A" if k == 1 else ("B" if k == 2 else str(k))
        print(f"   루프{tag}: {s:6.1f}~{e:6.1f}s | "
              f"PL 순yaw {pl_net:+7.1f}° | 다리 순yaw {leg_net:+7.1f}° | "
              f"PL 고유편차 {pl_net - leg_net:+7.1f}°")
    print(f"전체 최대 |PL-다리| 편차 : {np.max(np.abs(diverg)):.1f}°")
    hit_pl = np.any(pl_acc <= args.target)
    hit_leg = np.any(leg_acc <= args.target)
    print(f"{args.target:.0f}° 도달 여부 : Point-LIO={'예' if hit_pl else '아니오'}"
          f"  /  다리={'예' if hit_leg else '아니오'}")
    if hit_pl and not hit_leg:
        print("   → Point-LIO만 도달 : 처리(초기화/필터/extrinsic) 문제 의심")
    elif hit_pl and hit_leg:
        print("   → 둘 다 도달 : 입력/기하(degeneracy) 문제 의심")
    print("=" * 60)

    # ── 그림
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    # (1) 두 소스의 누적 yaw
    ax1.plot(pl_t, pl_acc, color="#D85A30", lw=1.6, label="Point-LIO yaw")
    ax1.plot(pl_t, leg_acc, color="#1D9E75", lw=1.6, label="Leg odom yaw")
    ax1.axhline(args.target, color="#A32D2D", ls="--", lw=1.0,
                label=f"target {args.target:.0f} deg")
    for s, e in still_segs:
        ax1.axvspan(s, e, color="0.85", zorder=0)
    for s, e in loops:
        ax1.axvline(s, color="0.6", ls=":", lw=0.8)
        ax1.axvline(e, color="0.6", ls=":", lw=0.8)
    # target 도달 지점 표시
    cross = np.where(pl_acc <= args.target)[0]
    if len(cross):
        i = cross[0]
        ax1.scatter([pl_t[i]], [pl_acc[i]], color="#A32D2D", zorder=5, s=30)
        ax1.annotate(f"{pl_t[i]:.1f}s", (pl_t[i], pl_acc[i]),
                     textcoords="offset points", xytext=(6, -12), fontsize=9)
    ax1.set_ylabel("accumulated yaw [deg]")
    ax1.set_title("Point-LIO vs leg odometry  (shaded = standstill)")
    ax1.legend(loc="best", fontsize=9)
    ax1.grid(alpha=0.3)

    # (2) 두 소스의 차 = Point-LIO 고유 편차
    ax2.plot(pl_t, diverg, color="#534AB7", lw=1.6, label="PL - leg (drift)")
    ax2.axhline(0, color="0.5", lw=0.8)
    for s, e in still_segs:
        ax2.axvspan(s, e, color="0.85", zorder=0)
    ax2.set_ylabel("yaw divergence [deg]")
    ax2.set_xlabel("time [s]")
    ax2.legend(loc="best", fontsize=9)
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"그림 저장: {args.out}")


if __name__ == "__main__":
    main()
