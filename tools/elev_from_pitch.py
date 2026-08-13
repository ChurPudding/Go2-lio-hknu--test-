#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
elev_from_pitch.py — IMU 피치 적분으로 고도 변화를 추정한다

왜 필요한가
-----------
다리 오도메트리의 position[2] 는 고도가 아니라 **지면 위 몸통 높이**다.
2026-08-12 실측: 경사로를 246 m 내려가고 184 m 올라왔는데 네 bag 모두
dz 가 ±0.01 m 였다. 즉 z 축은 지형을 전혀 반영하지 않는다.

/gnss 에도 고도 필드가 없다(fixed, hdop, lat, lon, satellite_*, timestamp).

원리
----
피치는 **중력 기준**이라 드리프트가 없다. yaw 는 적분값이라 분당 2~5도
밀리지만, roll·pitch 는 가속도계가 중력 방향을 계속 보고 있어 절대 기준이
있다. 따라서

    dz = sum( sin(pitch_i) * ds_i )

ds 는 오도메트리 이동거리(축척 k 적용), pitch 는 imu_state.rpy[1].

전제 — 반드시 검증할 것
-----------------------
몸통 피치 == 지면 경사 인가?
제어기가 몸통을 수평으로 유지하려 들면 경사에서도 피치가 0 에 가깝게
나오고, 그러면 이 방법은 성립하지 않는다. 아래 세 조건으로 판정한다.

    loop1_1449 (평지 폐루프)   dz ~ 0        <- 편향이 있으면 여기서 드러남
    slope_out  (내려감)        dz < 0
    slope_back (올라감)        dz > 0, 크기가 slope_out 과 비슷

사용
----
  python3 elev_from_pitch.py                    # 0812 bag 전부
  python3 elev_from_pitch.py --bag <경로> --plot
"""

import argparse
import glob
import math
import os
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser("~/fastlio_ws/tools"))
try:
    from go2_calib import K_OUTDOOR
except Exception:
    K_OUTDOOR = 1.23

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SMS = get_message("unitree_go/msg/SportModeState")
BAGDIR = os.path.expanduser("~/data/bags/outdoor/0812")


def load(bag_dir):
    """[t, x, y, roll, pitch, yaw, bodyheight]"""
    db = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    if not db:
        return None
    con = sqlite3.connect(db[0])
    cur = con.cursor()
    r = cur.execute("SELECT id FROM topics WHERE name='/sportmodestate'").fetchone()
    if not r:
        con.close()
        return None
    rows = cur.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (r[0],)).fetchall()
    con.close()
    out = []
    for ts, data in rows:
        m = deserialize_message(data, SMS)
        rpy = m.imu_state.rpy
        out.append((ts / 1e9, m.position[0], m.position[1],
                    rpy[0], rpy[1], rpy[2], m.position[2]))
    return np.array(out)


def analyse(d, k, stride):
    """반환 dict"""
    t = d[::stride, 0]
    xy = d[::stride, 1:3]
    pitch = d[::stride, 4]
    bh = d[::stride, 6]

    ds = np.linalg.norm(np.diff(xy, axis=0), axis=1) * k
    p_mid = 0.5 * (pitch[1:] + pitch[:-1])
    # Unitree rpy[1] 은 머리를 숙일 때 양수 (2026-08-13 확인:
    # slope_out 내리막/slope_back 오르막이 거울상으로 나옴)
    dz = -np.sin(p_mid) * ds
    z = np.concatenate([[0.0], np.cumsum(dz)])

    # 정지 구간(ds 아주 작음)은 피치 편향만 쌓이므로 따로 본다
    moving = ds > 1e-3
    return dict(t=t, z=z, dz=dz, ds=ds, pitch=pitch, bh=bh,
                s=np.concatenate([[0.0], np.cumsum(ds)]),
                L=float(ds.sum()),
                total=float(z[-1]),
                zmin=float(z.min()), zmax=float(z.max()),
                p_mean=float(np.degrees(pitch.mean())),
                p_std=float(np.degrees(pitch.std())),
                p_min=float(np.degrees(pitch.min())),
                p_max=float(np.degrees(pitch.max())),
                p_move_mean=float(np.degrees(p_mid[moving].mean()))
                if moving.any() else float("nan"),
                bh_mean=float(bh.mean()), bh_std=float(bh.std()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default=None)
    ap.add_argument("--k", type=float, default=K_OUTDOOR)
    ap.add_argument("--stride", type=int, default=10,
                    help="N 개마다 1개 사용. 292Hz 라 10 이면 약 29Hz")
    ap.add_argument("--dist", action="store_true",
                    help="x축을 시간 대신 누적거리(m)로. 지도 표시와 대조용")
    ap.add_argument("--mark", nargs="*", type=float, default=None,
                    help="세로선을 그을 누적거리들 (m)")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    bags = ([args.bag] if args.bag else
            sorted(d for d in glob.glob(os.path.join(BAGDIR, "go2_*"))
                   if glob.glob(os.path.join(d, "*.db3"))))
    if not bags:
        sys.exit("bag 을 찾지 못했습니다.")

    print(f"k = {args.k}, stride {args.stride}\n")
    print(f"{'bag':<28}{'경로장':>8}{'Δz':>8}{'z범위':>16}"
          f"{'피치평균':>10}{'피치범위':>16}{'몸통높이':>10}")
    print("-" * 98)

    results = {}
    for b in bags:
        d = load(b)
        if d is None or len(d) < 100:
            continue
        r = analyse(d, args.k, args.stride)
        name = os.path.basename(b).replace("go2_", "").replace("_0812", "")
        results[name] = r
        print(f"{name:<28}{r['L']:8.1f}{r['total']:+8.2f}"
              f"{r['zmin']:+8.2f}~{r['zmax']:+6.2f}"
              f"{r['p_mean']:+10.2f}"
              f"{r['p_min']:+8.2f}~{r['p_max']:+6.2f}"
              f"{r['bh_mean']:10.3f}")

    # ---------- 판정 ----------
    print("\n" + "=" * 60)
    print("검증")
    loop = next((v for kk, v in results.items() if "loop1" in kk), None)
    out = next((v for kk, v in results.items() if "slope_out" in kk), None)
    back = next((v for kk, v in results.items() if "slope_back" in kk), None)

    ok = True
    if loop:
        z = abs(loop["total"])
        bias = math.degrees(math.asin(max(min(loop["total"] / max(loop["L"], 1), 1), -1)))
        print(f"  평지 폐루프 Δz = {loop['total']:+.2f} m  "
              f"(경로장 {loop['L']:.0f} m, 등가 피치편향 {bias:+.3f}deg)")
        if z > 3.0:
            print("    ! 0 에서 크게 벗어남 — 피치에 상수 편향이 있을 수 있습니다")
            ok = False
        else:
            print("    OK — 편향이 작습니다")
    if out and back:
        print(f"  내리막 Δz = {out['total']:+.2f} m ({out['L']:.0f} m)")
        print(f"  오르막 Δz = {back['total']:+.2f} m ({back['L']:.0f} m)")
        if out["total"] < -0.5 and back["total"] > 0.5:
            print("    OK — 부호가 기대와 일치합니다")
        else:
            print("    ! 부호가 기대와 다릅니다. 제어기가 몸통을 수평으로")
            print("      유지하고 있을 가능성이 있습니다")
            ok = False

    print("\n" + ("=> 피치 적분이 유효해 보입니다. 다만 절대 검증(위성지도 등고선,\n"
                  "   기준점 실측)으로 한 번 더 대조하십시오."
                  if ok else
                  "=> 이 방법은 이 플랫폼에 맞지 않을 수 있습니다.\n"
                  "   대안: 라이다 지면 평면의 법선으로 실제 지면 경사를 구하기"))

    if args.plot and results:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            return
        n = len(results)
        fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=False)
        for name, r in results.items():
            tt = r["s"] if args.dist else (r["t"] - r["t"][0])
            ax[0].plot(tt, r["z"], lw=1.4, label=name)
            ax[1].plot(tt, np.degrees(r["pitch"]), lw=0.7, label=name)
        if args.mark:
            for a in ax:
                for mk in args.mark:
                    a.axvline(mk, ls="--", lw=1, color="orange", alpha=0.8)
        ax[0].axhline(0, ls="--", lw=0.8, color="gray")
        ax[0].set_ylabel("elevation [m]")
        ax[0].grid(alpha=0.3, ls=":")
        ax[0].legend(fontsize=8)
        ax[0].set_title("cumulative elevation from pitch integration")
        ax[1].axhline(0, ls="--", lw=0.8, color="gray")
        ax[1].set_xlabel("distance [m]" if args.dist else "t [s]")
        ax[1].set_ylabel("pitch [deg]")
        ax[1].grid(alpha=0.3, ls=":")
        ax[1].legend(fontsize=8)
        fig.tight_layout()
        p = os.path.expanduser(
            "~/fastlio_ws/results/outdoor_0812/figs/elev_from_pitch.png")
        os.makedirs(os.path.dirname(p), exist_ok=True)
        fig.savefig(p, dpi=140)
        print(f"\n저장: {p}")


if __name__ == "__main__":
    main()
