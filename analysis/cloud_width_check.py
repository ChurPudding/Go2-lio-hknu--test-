#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cloud_width_check.py

bag에서 /utlidar/cloud 의 프레임당 점 수(width)를 시간축으로 뽑아,
"되돌아올 때 입력 점이 줄어드는가"(효신 님 가설)를 검증한다.
다리 오도메트리(/utlidar/robot_odom)가 있으면 정지 구간도 함께 표시하고,
전반(직진 위주)/후반(복귀 위주) 평균 점 수를 비교 출력한다.

사용:
  python3 cloud_width_check.py ~/data/bags/ekf_test1
  python3 cloud_width_check.py <bag> --out width.png
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

CLOUD_TOPIC = "/utlidar/cloud"
LEG_TOPIC = "/utlidar/robot_odom"


def stamp_sec(msg):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if CLOUD_TOPIC not in types:
        sys.exit(f"[에러] {CLOUD_TOPIC} 없음. 담긴 토픽: {list(types)}")

    MsgCloud = get_message(types[CLOUD_TOPIC])
    have_leg = LEG_TOPIC in types
    MsgLeg = get_message(types[LEG_TOPIC]) if have_leg else None

    cloud = []   # (t, width = 점 수)
    leg = []     # (t, x, y)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == CLOUD_TOPIC:
            m = deserialize_message(data, MsgCloud)
            # width*height 가 실제 점 수. 보통 height=1 이라 width 와 같음.
            n = int(m.width) * int(m.height if m.height else 1)
            cloud.append((stamp_sec(m), n))
        elif have_leg and topic == LEG_TOPIC:
            m = deserialize_message(data, MsgLeg)
            p = m.pose.pose.position
            leg.append((stamp_sec(m), p.x, p.y))
    if not cloud:
        sys.exit(f"[에러] {CLOUD_TOPIC} 메시지가 0개입니다.")
    return np.array(cloud), (np.array(leg) if leg else None)


def still_mask(t, x, y, v_th):
    dt = np.clip(np.gradient(t), 1e-3, None)
    spd = np.hypot(np.gradient(x), np.gradient(y)) / dt
    return spd < v_th


def find_still_segments(t, mask, min_dur):
    segs, i, n = [], 0, len(t)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            if t[j] - t[i] >= min_dur:
                segs.append((t[i], t[j]))
            i = j + 1
        else:
            i += 1
    return segs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--still-speed", type=float, default=0.05)
    ap.add_argument("--min-still", type=float, default=3.0)
    ap.add_argument("--out", default="cloud_width.png")
    args = ap.parse_args()

    cloud, leg = read_bag(args.bag)
    t0 = cloud[0, 0]
    ct = cloud[:, 0] - t0
    cw = cloud[:, 1]

    # 정지 구간 (다리 오도가 있을 때만)
    still_segs = []
    if leg is not None:
        lt = leg[:, 0] - t0
        sm = still_mask(lt, leg[:, 1], leg[:, 2], args.still_speed)
        still_segs = find_still_segments(lt, sm, args.min_still)

    # 전반/후반 비교 (전체 시간의 절반 기준)
    half = ct[-1] / 2.0
    front = cw[ct < half]
    back = cw[ct >= half]

    print("=" * 56)
    print(f"bag              : {args.bag}")
    print(f"클라우드 프레임   : {len(cw)}")
    print(f"프레임당 점 수    : 최소 {cw.min():.0f} / 평균 {cw.mean():.0f} / 최대 {cw.max():.0f}")
    print(f"전반(0~{half:.0f}s)   평균 점 수 : {front.mean():.0f}")
    print(f"후반({half:.0f}s~끝) 평균 점 수 : {back.mean():.0f}")
    diff = (back.mean() - front.mean()) / front.mean() * 100
    print(f"후반이 전반 대비   : {diff:+.1f}%")
    if diff < -10:
        print("  → 되돌아올 때 입력 점이 뚜렷이 감소: '입력 빈약' 가설 지지")
    elif diff > 10:
        print("  → 되돌아올 때 오히려 증가: 입력 부족은 원인 아님")
    else:
        print("  → 큰 차이 없음: 입력 점 수는 원인 아닐 가능성 (정합/기하 쪽 의심)")
    print("=" * 56)

    # 그림
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(ct, cw, color="#2B6CB0", lw=0.7)
    # 이동 평균으로 추세선
    if len(cw) > 20:
        k = max(5, len(cw) // 100)
        kern = np.ones(k) / k
        ax.plot(ct, np.convolve(cw, kern, mode="same"),
                color="#D85A30", lw=1.6, label=f"moving avg ({k})")
    for s, e in still_segs:
        ax.axvspan(s, e, color="0.85", zorder=0)
    ax.axvline(half, color="0.5", ls="--", lw=1.0, label="front / back split")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("points per scan (/utlidar/cloud width)")
    ax.set_title("LiDAR input density over time  (shaded = standstill)")
    ax.legend(loc="best", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"그림 저장: {args.out}")


if __name__ == "__main__":
    main()
