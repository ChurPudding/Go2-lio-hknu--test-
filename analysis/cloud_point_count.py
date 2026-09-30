#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cloud_point_count.py — 원본 bag의 PointCloud2 프레임별 점 수를 뽑는다.

목적
  유리·거울 구간의 지문은 '점 수 급감'이다(빛이 통과/반사돼 되돌아온 점이 줆).
  프레임별 점 수를 시각축으로 보고, 급감하는 구간을 찾는다.
  급감 구간의 시각을 궤적 CSV와 맞추면 '어느 위치에서 점이 사라지나'가 나온다.

무엇을 하나
  1) bag에서 <topic>(기본 /utlidar/cloud)의 프레임별 점 수를 센다
     - 점 수 = width * height (PointCloud2 메타만 읽음, 역직렬화 없이 빠름)
  2) 점 수 시계열을 CSV로 저장 (t_sec, n_points)
  3) 급감 구간 자동 검출 (중앙값 대비 일정 비율 이하)
  4) --traj <궤적csv> 주면, 급감 시각의 위치(x,y)를 궤적에서 찾아 표시

시각
  bag 타임스탬프(t_ns) 사용, 첫 프레임=0 (원본 bag 시간, 1배).
  궤적 CSV(traj_to_csv_v3, 벽시계)와 맞추려면 --traj-rate 로 환산(기본 0.5).

사용
  python3 cloud_point_count.py ~/data/bags/lio_test_bag_loop_run1
  python3 cloud_point_count.py <bag> --topic /utlidar/cloud --traj aft_off25.csv --traj-rate 0.5
"""

import argparse
import os
import sys
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def resolve_bag(path):
    if os.path.isdir(path):
        for f in sorted(os.listdir(path)):
            if f.endswith(".db3"):
                return os.path.join(path, f)
        sys.exit(f"[에러] {path} 안에 .db3 없음")
    return path


def read_counts(bag, topic):
    db3 = resolve_bag(bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=db3, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        sys.exit(f"[에러] {topic} 없음. 있는 토픽: {list(types)}")
    MsgType = get_message(types[topic])

    ts, npts = [], []
    t0 = None
    while reader.has_next():
        tp, data, t_ns = reader.read_next()
        if tp != topic:
            continue
        m = deserialize_message(data, MsgType)
        t = t_ns * 1e-9
        if t0 is None:
            t0 = t
        n = int(m.width) * int(m.height)
        ts.append(t - t0)
        npts.append(n)
    return np.array(ts), np.array(npts)


def load_traj(path, rate):
    a = np.genfromtxt(path, delimiter=',', skip_header=1)
    return a[:, 0] * rate, a[:, 1], a[:, 2]   # bag시간, x, y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/utlidar/cloud")
    ap.add_argument("--drop-frac", type=float, default=0.6,
                    help="중앙값의 이 비율 미만이면 급감으로 본다 (기본 0.6)")
    ap.add_argument("--traj", default=None, help="궤적 CSV (급감 위치 표시용)")
    ap.add_argument("--traj-rate", type=float, default=0.5)
    ap.add_argument("--out", default=None, help="점수 시계열 CSV 저장 경로")
    args = ap.parse_args()

    t, n = read_counts(args.bag, args.topic)
    if len(n) == 0:
        sys.exit("[에러] 메시지 0개")

    med = float(np.median(n))
    thr = med * args.drop_frac
    print("=" * 60)
    print(f"토픽 {args.topic}   프레임 {len(n)}개")
    print(f"점 수: 중앙값 {med:.0f}, 최소 {n.min()}, 최대 {n.max()}")
    print(f"급감 임계: 중앙값의 {args.drop_frac:.0%} = {thr:.0f} 미만")
    print("=" * 60)

    # 급감 구간 (연속) 검출
    low = n < thr
    segs = []
    i = 0
    while i < len(low):
        if low[i]:
            j = i
            while j < len(low) and low[j]:
                j += 1
            segs.append((i, j - 1))
            i = j
        else:
            i += 1

    if not segs:
        print("급감 구간 없음 — 점 수가 고른 편. 유리 지문 약함.")
    else:
        print(f"급감 구간 {len(segs)}개 (bag 시간):")
        traj = load_traj(args.traj, args.traj_rate) if args.traj else None
        for (a, b) in segs:
            dur = t[b] - t[a]
            if dur < 0.3:      # 너무 짧은 건 노이즈로 무시
                continue
            nmin = n[a:b + 1].min()
            loc = ""
            if traj is not None:
                tt, xx, yy = traj
                # 급감 중앙 시각에 해당하는 궤적 위치
                tc = (t[a] + t[b]) / 2
                k = int(np.argmin(np.abs(tt - tc)))
                loc = f"  위치 ({xx[k]:+.1f}, {yy[k]:+.1f})"
            print(f"  {t[a]:6.1f}~{t[b]:6.1f}s ({dur:4.1f}s)  "
                  f"최소 {nmin:5d}점 (중앙 대비 {100*nmin/med:.0f}%){loc}")

    # 시계열 저장
    outpath = args.out or (os.path.splitext(os.path.basename(
        resolve_bag(args.bag)))[0] + "_ptcount.csv")
    np.savetxt(outpath, np.column_stack([t, n]), delimiter=",",
               header="t_sec_bag,n_points", comments="", fmt="%.4f,%d")
    print(f"\n점수 시계열 저장: {outpath}")

    # 그림 (matplotlib 있으면)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 4))
        ax.plot(t, n, lw=0.8)
        ax.axhline(med, color="g", ls=":", lw=0.8, label=f"median {med:.0f}")
        ax.axhline(thr, color="r", ls=":", lw=0.8, label=f"drop thr {thr:.0f}")
        ax.set_xlabel("t_sec (bag)")
        ax.set_ylabel("points per frame")
        ax.set_title(f"{args.topic} point count  (low = glass/degenerate suspect)")
        ax.legend(fontsize=8)
        fig.tight_layout()
        png = os.path.splitext(outpath)[0] + ".png"
        fig.savefig(png, dpi=130)
        print(f"그림 저장: {png}")
    except Exception as e:
        print(f"(그림 생략: {e})")


if __name__ == "__main__":
    main()
