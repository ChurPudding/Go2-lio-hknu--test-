#!/usr/bin/env python3
"""
plot_gnss_quality.py — /gnss 품질을 시간축과 궤적 위에 동시에 그린다.

목적: hdop 5.10 스파이크가 "언제" 났고 "어디서" 났는지를 한 장에서 본다.
  - 시작 직후에 몰려 있으면  -> 콜드스타트 수렴. 무시 가능.
  - 궤적의 특정 지점에 몰려 있으면 -> 그 지형이 multipath 원인. 순천에서 재현될 수 있음.

노드/로봇 불필요. bag만 있으면 된다.

사용:
    srcoff
    python3 plot_gnss_quality.py <bag.db3 또는 bag_dir> [출력.png]
"""

import json
import math
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


TOPIC = "/gnss"
HDOP_WARN = 2.0     # 이 위를 이상치로 표시
R_EARTH = 6378137.0


def read_gnss(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if TOPIC not in types:
        print(f"[FAIL] {TOPIC} 없음")
        sys.exit(1)
    cls = get_message(types[TOPIC])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))

    t, lat, lon, hdop, sats = [], [], [], [], []
    while reader.has_next():
        _, raw, t_ns = reader.read_next()
        try:
            d = json.loads(deserialize_message(raw, cls).data)
        except Exception:
            continue
        if not d.get("latitude") or not d.get("longitude"):
            continue
        t.append(t_ns / 1e9)
        lat.append(float(d["latitude"]))
        lon.append(float(d["longitude"]))
        hdop.append(float(d.get("hdop", 0.0)))
        sats.append(int(d.get("satellite_inuse", 0)))

    t = np.array(t)
    return t - t[0], np.array(lat), np.array(lon), np.array(hdop), np.array(sats)


def to_enu(lat, lon):
    """첫 점을 원점으로 한 국소 평면 좌표(m)."""
    lat0, lon0 = lat[0], lon[0]
    e = np.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    n = np.radians(lat - lat0) * R_EARTH
    return e, n


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    out = sys.argv[2] if len(sys.argv) > 2 else "gnss_quality.png"

    t, lat, lon, hdop, sats = read_gnss(sys.argv[1])
    e, n = to_enu(lat, lon)
    bad = hdop > HDOP_WARN

    fig = plt.figure(figsize=(13, 5.5))

    # ---- 좌: 시간축 -------------------------------------------------------
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.plot(t, hdop, lw=1.0, color="#378ADD", label="hdop")
    ax1.scatter(t[bad], hdop[bad], s=28, color="#E24B4A", zorder=5,
                label=f"hdop > {HDOP_WARN}")
    ax1.axhline(HDOP_WARN, ls="--", lw=0.7, color="#888780")
    ax1.set_xlabel("time since bag start [s]")
    ax1.set_ylabel("hdop")
    ax1.set_title("hdop over time")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.25)

    ax1b = ax1.twinx()
    ax1b.plot(t, sats, lw=0.8, color="#1D9E75", alpha=0.55)
    ax1b.set_ylabel("satellites in use", color="#1D9E75")
    ax1b.tick_params(axis="y", labelcolor="#1D9E75")

    # ---- 우: 궤적 ---------------------------------------------------------
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(e, n, lw=0.8, color="#B4B2A9", zorder=1)
    sc = ax2.scatter(e, n, c=hdop, s=16, cmap="viridis_r", zorder=2)
    ax2.scatter(e[bad], n[bad], s=70, facecolors="none",
                edgecolors="#E24B4A", lw=1.4, zorder=3,
                label=f"hdop > {HDOP_WARN}")
    ax2.scatter([e[0]], [n[0]], marker="s", s=60, color="#639922",
                zorder=4, label="start")
    ax2.scatter([e[-1]], [n[-1]], marker="X", s=70, color="#993C1D",
                zorder=4, label="end")
    fig.colorbar(sc, ax=ax2, label="hdop")
    ax2.set_xlabel("east [m]")
    ax2.set_ylabel("north [m]")
    ax2.set_title("trajectory colored by hdop")
    ax2.axis("equal")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"saved: {out}")

    # ---- 콘솔 요약 --------------------------------------------------------
    print()
    print(f"총 {len(t)} 점, {t[-1]:.1f} s")
    print(f"hdop > {HDOP_WARN} 인 점: {bad.sum()} 개")
    if bad.sum():
        print()
        print("  t[s]     hdop  sats    east[m]   north[m]")
        for i in np.where(bad)[0]:
            print(f"  {t[i]:7.1f}  {hdop[i]:5.2f}  {sats[i]:3d}"
                  f"  {e[i]:9.1f} {n[i]:9.1f}")

        first30 = (t[bad] < 30).sum()
        print()
        print(f"  이 중 시작 30초 이내: {first30} / {bad.sum()}")
        if first30 == bad.sum():
            print("  -> 전부 초기 수렴 구간. 주행 중에는 문제 없음.")
        else:
            print("  -> 주행 중에도 발생. 위 좌표를 지도와 대조해 지형을 확인할 것.")

    # 시작-끝 거리 (루프 폐합 확인)
    print()
    print(f"시작-끝 거리: {math.hypot(e[-1]-e[0], n[-1]-n[0]):.1f} m")


if __name__ == "__main__":
    main()
