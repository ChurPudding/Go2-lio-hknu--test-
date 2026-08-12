#!/usr/bin/env python3
"""
plot_traj.py -- 실외 bag 궤적 시각화 (오프라인, DDS 미사용)

  python3 plot_traj.py [bag_dir ...]

인자를 주지 않으면 ~/data/bags/outdoor/0812 와 ~ 아래의
go2_* / gps_static_* 디렉터리를 자동으로 찾는다.

출력: <출력폴더>/traj_<bagname>.png  각 bag 개별
      <출력폴더>/traj_all.png        전체 겹쳐보기
"""

import glob
import json
import math
import os
import sqlite3
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib 이 없습니다:  pip3 install matplotlib")

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SMS = get_message("unitree_go/msg/SportModeState")
STR = get_message("std_msgs/msg/String")

OUTDIR = os.path.expanduser("~/data/bags/outdoor/0812/plots")


# ----------------------------------------------------------------------
# bag 읽기
# ----------------------------------------------------------------------
def load_bag(d):
    """(odo, gps) 반환.  odo=[t,x,y,z,yaw]  gps=[t,lat,lon,hdop]"""
    db = glob.glob(os.path.join(d, "*.db3"))
    if not db:
        return None, None

    con = sqlite3.connect(db[0])
    cur = con.cursor()

    def grab(topic):
        r = cur.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
        if not r:
            return []
        return cur.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (r[0],),
        ).fetchall()

    odo = []
    for ts, data in grab("/sportmodestate"):
        m = deserialize_message(data, SMS)
        odo.append((ts / 1e9, m.position[0], m.position[1],
                    m.position[2], m.imu_state.rpy[2]))

    gps = []
    for ts, data in grab("/gnss"):
        try:
            j = json.loads(deserialize_message(data, STR).data)
        except Exception:
            continue
        if j.get("fixed") != 1:
            continue
        gps.append((ts / 1e9, j["latitude"], j["longitude"], j.get("hdop", 0.0)))

    con.close()
    return np.array(odo), (np.array(gps) if gps else np.zeros((0, 4)))


def to_enu(gps):
    """위경도 -> 국소 ENU(m).  첫 fix 를 원점으로."""
    lat0, lon0 = gps[0, 1], gps[0, 2]
    e = (gps[:, 2] - lon0) * 111320.0 * math.cos(math.radians(lat0))
    n = (gps[:, 1] - lat0) * 110540.0
    return np.column_stack([e, n])


def align(src, dst):
    """src -> dst 로 yaw 회전 + 평행이동 (Umeyama, 스케일 고정).
    반환: 변환된 src, ENU 기준 yaw(deg), 잔차 배열"""
    sc, dc = src.mean(0), dst.mean(0)
    a, b = src - sc, dst - dc
    u, _, vt = np.linalg.svd(a.T @ b)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    out = (src - sc) @ r.T + dc
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    return out, yaw, np.linalg.norm(dst - out, axis=1)


def path_len(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


# ----------------------------------------------------------------------
# 그리기
# ----------------------------------------------------------------------
def draw_one(ax, name, odo, gps):
    """단일 bag 을 축 하나에 그린다. 요약 dict 반환."""
    info = {"name": name}
    O = odo[:, 1:3]
    L = path_len(O)
    err = float(np.linalg.norm(O[-1] - O[0]))
    info["odo_len"] = L
    info["odo_err"] = err
    info["odo_pct"] = err / L * 100 if L > 1 else float("nan")

    has_gps = len(gps) >= 10

    if has_gps:
        P = to_enu(gps)
        # GPS 시각에 오도메트리 보간
        ox = np.interp(gps[:, 0], odo[:, 0], odo[:, 1])
        oy = np.interp(gps[:, 0], odo[:, 0], odo[:, 2])
        Oi = np.column_stack([ox, oy])
        Oa, yaw, res = align(Oi, P)

        # 전체 오도메트리도 같은 변환으로
        sc, dc = Oi.mean(0), P.mean(0)
        th = math.radians(yaw)
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        Ofull = (O - sc) @ R.T + dc

        t = gps[:, 0] - gps[0, 0]
        head = P[t <= 10]
        tail = P[t >= t[-1] - 10]
        info["gps_len"] = path_len(P)
        info["gps_err"] = float(np.linalg.norm(tail.mean(0) - head.mean(0)))
        info["hdop"] = float(gps[:, 3].mean())
        info["yaw_enu"] = yaw
        info["res_rms"] = float(np.sqrt((res ** 2).mean()))

        ax.plot(P[:, 0], P[:, 1], "-", lw=1.0, color="tab:orange",
                alpha=0.8, label="GPS (ENU)")
        ax.plot(Ofull[:, 0], Ofull[:, 1], "-", lw=1.6, color="tab:blue",
                label="Leg odometry")
        S, E = Ofull[0], Ofull[-1]
    else:
        info["gps_len"] = 0.0
        ax.plot(O[:, 0], O[:, 1], "-", lw=1.6, color="tab:blue",
                label="Leg odometry (no GPS fix)")
        S, E = O[0], O[-1]

    ax.plot(*S, "o", ms=9, color="green", zorder=5, label="Start")
    ax.plot(*E, "s", ms=9, color="red", zorder=5, label="End")
    ax.plot([S[0], E[0]], [S[1], E[1]], ":", lw=1.2, color="crimson")

    title = f"{name}\n{L:.1f} m,  gap {err:.2f} m"
    if L > 1:
        title += f"  ({info['odo_pct']:.2f}%)"
    if has_gps:
        title += f"\nGPS gap {info['gps_err']:.2f} m,  align RMS {info['res_rms']:.2f} m"
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("East [m]" if has_gps else "x [m]", fontsize=8)
    ax.set_ylabel("North [m]" if has_gps else "y [m]", fontsize=8)
    ax.axis("equal")
    ax.grid(alpha=0.3, ls=":")
    ax.legend(fontsize=7, loc="best")
    return info


def main():
    args = sys.argv[1:]
    if args:
        dirs = [os.path.abspath(a) for a in args]
    else:
        pats = []
        for base in (os.path.expanduser("~/data/bags/outdoor/0812"),
                     os.path.expanduser("~")):
            pats += glob.glob(os.path.join(base, "go2_*"))
            pats += glob.glob(os.path.join(base, "gps_static_*"))
        seen, dirs = set(), []
        for p in sorted(pats):
            b = os.path.basename(p)
            if os.path.isdir(p) and b not in seen and glob.glob(os.path.join(p, "*.db3")):
                seen.add(b)
                dirs.append(p)

    if not dirs:
        sys.exit("bag 을 찾지 못했습니다. 경로를 인자로 주십시오.")

    os.makedirs(OUTDIR, exist_ok=True)
    results, loaded = [], []

    for d in dirs:
        name = os.path.basename(d)
        odo, gps = load_bag(d)
        if odo is None or len(odo) < 2:
            print(f"[건너뜀] {name}  (/sportmodestate 없음)")
            continue
        loaded.append((name, odo, gps))

        fig, ax = plt.subplots(figsize=(6.5, 6.5))
        info = draw_one(ax, name, odo, gps)
        fig.tight_layout()
        out = os.path.join(OUTDIR, f"traj_{name}.png")
        fig.savefig(out, dpi=140)
        plt.close(fig)
        results.append(info)
        print(f"[저장] {out}")

    # 전체 격자
    if loaded:
        n = len(loaded)
        cols = min(3, n)
        rows = math.ceil(n / cols)
        fig, axes = plt.subplots(rows, cols, figsize=(6.0 * cols, 5.8 * rows))
        axes = np.atleast_1d(axes).ravel()
        for ax, (name, odo, gps) in zip(axes, loaded):
            draw_one(ax, name, odo, gps)
        for ax in axes[n:]:
            ax.axis("off")
        fig.tight_layout()
        out = os.path.join(OUTDIR, "traj_all.png")
        fig.savefig(out, dpi=130)
        plt.close(fig)
        print(f"[저장] {out}")

    # 요약표
    print("\n" + "=" * 78)
    print(f"{'bag':<28}{'odo_len':>9}{'odo_gap':>9}{'drift%':>8}"
          f"{'gps_gap':>9}{'RMS':>7}{'hdop':>6}")
    print("-" * 78)
    for r in results:
        print(f"{r['name']:<28}{r['odo_len']:>9.1f}{r['odo_err']:>9.2f}"
              f"{r['odo_pct']:>8.2f}"
              f"{r.get('gps_err', float('nan')):>9.2f}"
              f"{r.get('res_rms', float('nan')):>7.2f}"
              f"{r.get('hdop', float('nan')):>6.2f}")
    print("=" * 78)
    print("주의: 편도 구간(slope_*)의 gap/drift 는 폐루프가 아니므로 의미 없음")


if __name__ == "__main__":
    main()
