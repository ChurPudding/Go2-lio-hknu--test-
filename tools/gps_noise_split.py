#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_noise_split.py — GPS 오차를 '빠른 떨림' 과 '느린 흐름' 으로 나눠 잰다

왜 나누는가
-----------
GPS factor 는 각 시점의 오차가 서로 독립이라고 가정한다. 그 가정이 맞는 것은
빠른 떨림뿐이고, 분 단위로 서서히 밀리는 흐름에는 맞지 않는다. 후자는 모든
factor 가 같은 방향으로 당기므로 최적화가 통째로 끌려간다.

그래서 sigma 를 떨림 크기가 아니라 흐름까지 감싸는 값으로 잡아야 한다.
이 스크립트는 두 성분을 각각 재서 그 근거를 만든다.

측정 방법
---------
정지 bag (로봇이 움직이지 않음) 이 전제다. 참값이 상수이므로 관측된 변화가
곧 오차다.

  빠른 떨림   인접 fix 간 이동거리.  참값이 0 이므로 전부 노이즈.
              Allan 편차식으로 sigma_fast 를 추정한다.
  느린 흐름   win 초 이동평균을 낸 뒤 그 평균값이 시간에 따라 얼마나
              이동하는지 본다. 떨림은 평균에서 지워지므로 흐름만 남는다.
  전체        전 구간 평균으로부터의 RMS. GTSAM sigma 의 현실적 하한.

출력
----
  sigma_fast, sigma_slow, sigma_total 과 권장 GPS factor sigma

사용
----
  python3 gps_noise_split.py                       # gps_static_0812_1415
  python3 gps_noise_split.py <bag> [--win 10]
  python3 gps_noise_split.py <bag> --plot          # PNG 저장
"""

import argparse
import glob
import json
import math
import os
import sqlite3
import sys

import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

STR = get_message("std_msgs/msg/String")
DEFAULT = os.path.expanduser("~/data/bags/outdoor/0812/gps_static_0812_1415")


def load_gps(bag_dir, require_fix=True):
    db = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    if not db:
        sys.exit(f"db3 없음: {bag_dir}")
    con = sqlite3.connect(db[0])
    cur = con.cursor()
    r = cur.execute("SELECT id FROM topics WHERE name='/gnss'").fetchone()
    if not r:
        sys.exit("/gnss 토픽이 없습니다.")
    rows = cur.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (r[0],)
    ).fetchall()
    con.close()

    out, nfix0 = [], 0
    for ts, data in rows:
        try:
            j = json.loads(deserialize_message(data, STR).data)
        except Exception:
            continue
        if j.get("fixed") != 1:
            nfix0 += 1
            if require_fix:
                continue
        out.append((ts / 1e9, j["latitude"], j["longitude"],
                    j.get("hdop", 0.0), j.get("satellite_inuse", 0)))
    if nfix0:
        print(f"  (fixed!=1 인 {nfix0}개는 제외)")
    if len(out) < 30:
        sys.exit(f"쓸 수 있는 fix 가 {len(out)}개뿐입니다.")
    return np.array(out)


def to_enu(g):
    lat0, lon0 = g[:, 1].mean(), g[:, 2].mean()
    e = (g[:, 2] - lon0) * 111320.0 * math.cos(math.radians(lat0))
    n = (g[:, 1] - lat0) * 110540.0
    return np.column_stack([e, n])


def moving_average(t, P, win):
    """win 초 창의 이동평균. (중심 정렬)"""
    out = np.empty_like(P)
    for i, ti in enumerate(t):
        m = np.abs(t - ti) <= win / 2.0
        out[i] = P[m].mean(0)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", nargs="?", default=DEFAULT)
    ap.add_argument("--win", type=float, default=10.0,
                    help="느린 흐름을 볼 때 쓰는 이동평균 창 (초)")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    print(f"bag: {os.path.basename(args.bag)}")
    g = load_gps(args.bag)
    t = g[:, 0] - g[0, 0]
    P = to_enu(g)
    dur = t[-1]
    rate = len(t) / dur if dur > 0 else 0

    print(f"  fix {len(t)}개, {dur:.0f}s, {rate:.2f} Hz")
    print(f"  hdop {g[:,3].mean():.2f} (최소 {g[:,3].min():.2f} 최대 {g[:,3].max():.2f})")
    print(f"  위성 inuse 평균 {g[:,4].mean():.1f}")

    # 정지 확인
    span = np.linalg.norm(P.max(0) - P.min(0))
    print(f"  좌표 전체 퍼짐 {span:.2f} m")

    # ---------- 1) 빠른 떨림 ----------
    d = np.diff(P, axis=0)
    step = np.linalg.norm(d, axis=1)
    # 인접 차의 분산 = 2 * sigma^2  (두 독립 관측의 차)
    sigma_fast = float(np.sqrt((d ** 2).sum(1).mean() / 2.0))

    # ---------- 2) 느린 흐름 ----------
    S = moving_average(t, P, args.win)
    center = P.mean(0)
    slow = np.linalg.norm(S - center, axis=1)
    sigma_slow = float(np.sqrt(((S - center) ** 2).sum(1).mean()))
    slow_span = float(np.linalg.norm(S.max(0) - S.min(0)))
    # 처음/끝 10초 평균의 이동 = 전체 표류
    head = P[t <= 10].mean(0)
    tail = P[t >= dur - 10].mean(0)
    total_drift = float(np.linalg.norm(tail - head))

    # ---------- 3) 전체 ----------
    sigma_total = float(np.sqrt(((P - center) ** 2).sum(1).mean()))

    print("\n" + "=" * 56)
    print(f"  빠른 떨림  sigma_fast  = {sigma_fast:6.2f} m   "
          f"(인접 fix 간 평균 {step.mean():.2f} m)")
    print(f"  느린 흐름  sigma_slow  = {sigma_slow:6.2f} m   "
          f"({args.win:.0f}s 평균 기준, 이동폭 {slow_span:.2f} m)")
    print(f"  전체       sigma_total = {sigma_total:6.2f} m")
    print(f"  처음10s -> 마지막10s 평균 이동 = {total_drift:.2f} m")
    print("=" * 56)

    # ---------- 권장값 ----------
    rec = max(sigma_total, sigma_fast) * 1.5
    print("\nGTSAM GPS factor 권장 sigma")
    print(f"  보수적(안전)  {max(rec, 3.0):.1f} m   <- 우선 이 값으로 시작")
    print(f"  공격적        {sigma_total:.1f} m   (GPS 를 더 믿음)")
    print("  주의: 이 bag 은 콜드 스타트 직후라 느린 흐름이 과대평가됐을 수 있습니다.")
    print("        위성 7개 이상 수렴한 뒤 정지 bag 을 다시 뜨면 값이 내려갑니다.")

    if sigma_slow > sigma_fast:
        print("\n  ** 느린 흐름이 빠른 떨림보다 큽니다. **")
        print("     GPS factor 의 독립 가정이 깨지는 구간이므로")
        print("     sigma 를 떨림 기준으로 잡으면 최적화가 끌려갑니다.")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("\nmatplotlib 없음 — 그림 생략")
            return
        fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
        ax[0].plot(P[:, 0] - center[0], P[:, 1] - center[1], ".",
                   ms=2, alpha=0.5, label="raw fix")
        ax[0].plot(S[:, 0] - center[0], S[:, 1] - center[1], "-",
                   lw=1.6, color="tab:red", label=f"{args.win:.0f}s mean")
        ax[0].set_aspect("equal")
        ax[0].grid(alpha=0.3, ls=":")
        ax[0].set_xlabel("East [m]")
        ax[0].set_ylabel("North [m]")
        ax[0].set_title("scatter (centered)")
        ax[0].legend(fontsize=8)

        ax[1].plot(t, P[:, 0] - center[0], lw=0.8, label="East")
        ax[1].plot(t, P[:, 1] - center[1], lw=0.8, label="North")
        ax[1].plot(t, S[:, 0] - center[0], lw=2, color="tab:blue", alpha=0.6)
        ax[1].plot(t, S[:, 1] - center[1], lw=2, color="tab:orange", alpha=0.6)
        ax[1].grid(alpha=0.3, ls=":")
        ax[1].set_xlabel("t [s]")
        ax[1].set_ylabel("offset [m]")
        ax[1].set_title("time series (thick = smoothed)")
        ax[1].legend(fontsize=8)

        ax[2].hist(step, bins=40, color="tab:green", alpha=0.8)
        ax[2].axvline(step.mean(), color="crimson", lw=1.5)
        ax[2].grid(alpha=0.3, ls=":")
        ax[2].set_xlabel("distance between adjacent fixes [m]")
        ax[2].set_ylabel("count")
        ax[2].set_title(f"fast jitter (mean {step.mean():.2f} m)")

        fig.tight_layout()
        out = os.path.join(os.path.dirname(os.path.abspath(args.bag)),
                           "gps_noise_split.png")
        fig.savefig(out, dpi=140)
        print(f"\n저장: {out}")


if __name__ == "__main__":
    main()
