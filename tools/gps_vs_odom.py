#!/usr/bin/env python3
"""
gps_vs_odom.py — hdop 불량 구간에서 GPS 위치가 실제로 틀어졌는지 판정한다.

방법
----
leg odom (0.31% 드리프트)을 기준으로 삼는다. 100m 주행에 31cm이므로
GPS의 미터급 오차를 재기에 충분하다.

  1. /gnss 와 /sportmodestate 를 시간으로 매칭
  2. **hdop 정상 구간만 써서** 상사변환(회전+평행이동+스케일) 추정
     - 불량 구간을 정렬에 넣으면 오차가 정렬에 흡수되어 보이지 않게 된다
  3. 그 변환을 전 구간에 적용하고 잔차를 본다
  4. 불량 구간의 잔차가 정상 구간과 같으면 -> hdop이 나빠도 위치는 멀쩡
     불량 구간만 잔차가 크면      -> 실제 multipath 변위. 방어 필요

사용:
    srcoff
    python3 gps_vs_odom.py <bag.db3> [출력.png]
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


GNSS_TOPIC = "/gnss"
ODOM_TOPIC = "/sportmodestate"
HDOP_WARN = 2.0
R_EARTH = 6378137.0
MAX_DT = 0.6          # GPS-odom 매칭 허용 시간차 [s]


def read_topics(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in (GNSS_TOPIC, ODOM_TOPIC):
        if need not in types:
            print(f"[FAIL] {need} 없음. 녹화된 토픽: {sorted(types)}")
            sys.exit(1)

    gnss_cls = get_message(types[GNSS_TOPIC])
    odom_cls = get_message(types[ODOM_TOPIC])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[GNSS_TOPIC, ODOM_TOPIC]))

    g_t, g_lat, g_lon, g_hdop, g_sat = [], [], [], [], []
    o_t, o_xy = [], []

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns / 1e9
        if topic == GNSS_TOPIC:
            try:
                d = json.loads(deserialize_message(raw, gnss_cls).data)
            except Exception:
                continue
            if not d.get("latitude") or not d.get("longitude"):
                continue
            g_t.append(t)
            g_lat.append(float(d["latitude"]))
            g_lon.append(float(d["longitude"]))
            g_hdop.append(float(d.get("hdop", 0.0)))
            g_sat.append(int(d.get("satellite_inuse", 0)))
        else:
            m = deserialize_message(raw, odom_cls)
            o_t.append(t)
            o_xy.append([m.position[0], m.position[1]])

    return (np.array(g_t), np.array(g_lat), np.array(g_lon),
            np.array(g_hdop), np.array(g_sat),
            np.array(o_t), np.array(o_xy))


def to_enu(lat, lon):
    lat0, lon0 = lat[0], lon[0]
    e = np.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    n = np.radians(lat - lat0) * R_EARTH
    return np.column_stack([e, n])


def umeyama_2d(src, dst):
    """src -> dst 상사변환. 반환 (s, R, t)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    C = D.T @ S / len(src)
    U, sig, Vt = np.linalg.svd(C)
    F = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        F[1, 1] = -1
    R = U @ F @ Vt
    s = float(np.trace(np.diag(sig) @ F) / (S ** 2).sum() * len(src))
    t = mu_d - s * R @ mu_s
    return s, R, t


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    out = sys.argv[2] if len(sys.argv) > 2 else "gps_vs_odom.png"

    g_t, lat, lon, hdop, sat, o_t, o_xy = read_topics(sys.argv[1])
    gps = to_enu(lat, lon)

    # --- GPS 시각에 가장 가까운 odom 샘플 매칭 ---------------------------
    idx = np.searchsorted(o_t, g_t)
    idx = np.clip(idx, 1, len(o_t) - 1)
    left = np.abs(o_t[idx - 1] - g_t)
    right = np.abs(o_t[idx] - g_t)
    idx = np.where(left < right, idx - 1, idx)
    dt = np.abs(o_t[idx] - g_t)

    ok = dt < MAX_DT
    if ok.sum() < 20:
        print(f"[FAIL] 매칭된 쌍 {ok.sum()}개. 시간축 확인 필요")
        sys.exit(1)

    gps, odom = gps[ok], o_xy[idx[ok]]
    hdop, sat, t = hdop[ok], sat[ok], g_t[ok] - g_t[0]
    bad = hdop > HDOP_WARN
    good = ~bad

    print(f"매칭 {ok.sum()} / {len(g_t)} 쌍  (평균 dt {dt[ok].mean()*1000:.0f} ms)")
    print(f"hdop 정상 {good.sum()} 개, 불량 {bad.sum()} 개")

    # --- 정상 구간만으로 정렬 ---------------------------------------------
    s, R, tr = umeyama_2d(odom[good], gps[good])
    print(f"\n추정 스케일 k = {s:.4f}   (앞서 확인한 1.19~1.23 범위와 비교)")
    print(f"추정 yaw      = {math.degrees(math.atan2(R[1,0], R[0,0])):.2f} deg")

    odom_al = (s * (R @ odom.T)).T + tr
    res = np.linalg.norm(gps - odom_al, axis=1)

    r_good, r_bad = res[good], res[bad]
    print()
    print(f"잔차 (정상 구간)  평균 {r_good.mean():.2f} m,  "
          f"중앙 {np.median(r_good):.2f} m,  최대 {r_good.max():.2f} m")
    if bad.sum():
        print(f"잔차 (불량 구간)  평균 {r_bad.mean():.2f} m,  "
              f"중앙 {np.median(r_bad):.2f} m,  최대 {r_bad.max():.2f} m")
        ratio = r_bad.mean() / max(r_good.mean(), 1e-6)
        print()
        if ratio > 2.0:
            print(f"  -> 불량 구간 잔차가 {ratio:.1f}배. 실제 위치 변위 발생.")
            print("     hdop 가중만으로는 부족. Huber + 위성수 게이트 필요.")
        elif ratio > 1.3:
            print(f"  -> 불량 구간 잔차가 {ratio:.1f}배. 완만한 열화.")
            print("     현재 공분산 모델로 흡수 가능한 수준.")
        else:
            print(f"  -> 불량 구간 잔차가 {ratio:.1f}배. 유의한 차이 없음.")
            print("     hdop은 나빴지만 위치는 멀쩡. 과도한 방어는 불필요.")

    # --- 그림 -------------------------------------------------------------
    fig = plt.figure(figsize=(13, 5.5))

    ax1 = fig.add_subplot(1, 2, 1)
    ax1.plot(t, res, lw=1.0, color="#378ADD", label="|GPS - odom| [m]")
    if bad.sum():
        ax1.scatter(t[bad], res[bad], s=30, color="#E24B4A", zorder=5,
                    label=f"hdop > {HDOP_WARN}")
    ax1.axhline(r_good.mean(), ls="--", lw=0.8, color="#639922",
                label=f"good mean {r_good.mean():.2f} m")
    ax1.set_xlabel("time since bag start [s]")
    ax1.set_ylabel("residual [m]")
    ax1.set_title("GPS residual against leg odom")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.25)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(odom_al[:, 0], odom_al[:, 1], lw=1.2, color="#639922",
             label="leg odom (aligned)", zorder=2)
    sc = ax2.scatter(gps[:, 0], gps[:, 1], c=res, s=16, cmap="magma_r", zorder=3)
    if bad.sum():
        ax2.scatter(gps[bad, 0], gps[bad, 1], s=70, facecolors="none",
                    edgecolors="#E24B4A", lw=1.4, zorder=4,
                    label=f"hdop > {HDOP_WARN}")
    fig.colorbar(sc, ax=ax2, label="residual [m]")
    ax2.set_xlabel("east [m]")
    ax2.set_ylabel("north [m]")
    ax2.set_title("GPS points colored by residual")
    ax2.axis("equal")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"\nsaved: {out}")


if __name__ == "__main__":
    main()
