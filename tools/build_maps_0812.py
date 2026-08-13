#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_maps_0812.py — 실외 bag 여러 개를 한 번에 점군·격자지도로 만든다
                     GPS 보정 유무를 골라서 각각 생성한다

보정 방식 세 가지
-----------------
none   오도메트리 그대로 + 고정 축척 k(=1.1995).  odom_map_build_v3 와 동일.
       좌표계는 로봇 부팅 시점 기준이라 bag 마다 방위가 다르다.

rigid  GPS ENU 궤적에 상사변환(yaw 회전 + 평행이동 + 축척)으로 한 번 맞춘다.
       - 축척을 고정 k 가 아니라 **그 bag 의 GPS 로부터** 구한다
       - 결과 좌표계가 ENU(x=동, y=북) 가 되어 모든 bag 이 같은 방위로 정렬된다
       - GPS 순간 노이즈가 지도에 섞이지 않는다 (전역 변환 하나뿐)

warp   rigid 위에 잔차를 시간축으로 완만하게 보정한다.
       잔차 r(t) = GPS − 정렬된오도메트리 를 가우시안(σ초)으로 매끈하게 만든 뒤
       궤적에 더한다. 저주파만 남기므로 GPS 노이즈(2~5m)가 그대로 들어가지 않는다.
       loop_correct_v2 의 '증분 궤적 변형'과 같은 발상이다.

  주의: 축척은 위치 t(t) 에만 적용한다. 스캔 자체(p_body)는 건드리지 않는다.
        v2 주석의 원리와 동일하다.

사용
----
  # 전부, 세 방식 모두
  python3 build_maps_0812.py

  # 특정 bag 만
  python3 build_maps_0812.py --bags go2_loop1_0812_1449 go2_loop1_0812_1440

  # 방식 지정, 격자 해상도 15cm
  python3 build_maps_0812.py --modes rigid warp --grid-res 0.15

  # 빠른 시험 (3프레임에 1개만)
  python3 build_maps_0812.py --stride 3
"""

import argparse
import glob
import json
import math
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser("~/fastlio_ws/tools"))

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d 가 없습니다.")

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# odom_map_build_v3 의 검증된 함수를 그대로 재사용한다
try:
    from odom_map_build_v3 import (cloud_xyz, get_extrinsic, load_odom,
                                   nearest_odom, open_bag, quat_to_R, topic_id)
except ImportError as e:
    sys.exit(f"odom_map_build_v3.py 를 불러오지 못했습니다: {e}\n"
             f"(cloud_xyz 패치가 적용되어 있어야 합니다)")

STR = get_message("std_msgs/msg/String")

BAGDIR = os.path.expanduser("~/data/bags/outdoor/0812")
OUTDIR = os.path.expanduser("~/fastlio_ws/results/outdoor_0812")
GRID = os.path.expanduser("~/fastlio_ws/tools/pcd_to_grid.py")

CLOUD_TOPIC = "/utlidar/cloud"


# ----------------------------------------------------------------------
# GPS
# ----------------------------------------------------------------------

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from go2_calib import K_OUTDOOR

def _m_per_deg(lat_deg):
    """위도별 1도당 미터. WGS84 근사 (Snyder).
    이전 상수 110540/111320 은 위도 37도에서 각각 0.40%/0.12% 작았다."""
    p = math.radians(lat_deg)
    m_lat = 111132.92 - 559.82 * math.cos(2 * p) + 1.175 * math.cos(4 * p)
    m_lon = 111412.84 * math.cos(p) - 93.5 * math.cos(3 * p)
    return m_lat, m_lon


def load_gps(cur):
    """fixed==1 인 것만.  [t, lat, lon, hdop]"""
    tid = topic_id(cur, "/gnss")
    if tid is None:
        return np.zeros((0, 4))
    rows = cur.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
    ).fetchall()
    out = []
    for ts, data in rows:
        try:
            j = json.loads(deserialize_message(data, STR).data)
        except Exception:
            continue
        if j.get("fixed") != 1:
            continue
        out.append((ts / 1e9, j["latitude"], j["longitude"], j.get("hdop", 0.0)))
    return np.array(out) if out else np.zeros((0, 4))


def to_enu(gps, lat0=None, lon0=None):
    if lat0 is None:
        lat0, lon0 = gps[0, 1], gps[0, 2]
    _MLAT, _MLON = _m_per_deg(lat0)
    e = (gps[:, 2] - lon0) * _MLON
    n = (gps[:, 1] - lat0) * _MLAT
    return np.column_stack([e, n]), lat0, lon0


def similarity_2d(src, dst):
    """src -> dst 최적 상사변환.  반환 (yaw_rad, scale, translation(2,))"""
    sc, dc = src.mean(0), dst.mean(0)
    a, b = src - sc, dst - dc
    u, s_val, vt = np.linalg.svd(a.T @ b)
    R = vt.T @ u.T
    if np.linalg.det(R) < 0:
        vt[-1] *= -1
        R = vt.T @ u.T
    var = (a ** 2).sum()
    s = s_val.sum() / var if var > 1e-9 else 1.0
    t = dc - s * (R @ sc)
    return math.atan2(R[1, 0], R[0, 0]), float(s), t


def smooth(series, t, sigma_s):
    """불규칙 시각 series 를 가우시안으로 매끈하게. (N,2) -> (N,2)"""
    if sigma_s <= 0:
        return series
    out = np.empty_like(series)
    for i, ti in enumerate(t):
        w = np.exp(-0.5 * ((t - ti) / sigma_s) ** 2)
        w /= w.sum()
        out[i] = (series * w[:, None]).sum(0)
    return out


# ----------------------------------------------------------------------
def build(bag_dir, mode, args):
    """한 bag 을 한 방식으로 누적. 성공하면 pcd 경로 반환."""
    name = os.path.basename(bag_dir)
    tag = f"{name}_{mode}"
    print(f"\n{'='*62}\n{tag}")

    R_BL, lever = get_extrinsic(False)
    con = open_bag(bag_dir)
    cur = con.cursor()
    odo = load_odom(cur)

    # ---------- 궤적 보정 계획 ----------
    yaw = 0.0
    k = args.k
    T = np.zeros(2)
    warp_t = warp_v = None
    frame = "odom"

    if mode != "none":
        gps = load_gps(cur)
        if len(gps) < 20:
            print(f"  GPS fix {len(gps)}개 — 건너뜀 (fix 부족)")
            con.close()
            return None
        P, lat0, lon0 = to_enu(gps)
        ox = np.interp(gps[:, 0], odo[:, 0], odo[:, 1])
        oy = np.interp(gps[:, 0], odo[:, 0], odo[:, 2])
        O = np.column_stack([ox, oy])

        yaw, k, T = similarity_2d(O, P)
        Rz = np.array([[math.cos(yaw), -math.sin(yaw)],
                       [math.sin(yaw), math.cos(yaw)]])
        fitted = k * (O @ Rz.T) + T
        res = P - fitted
        rms = float(np.sqrt((res ** 2).sum(1).mean()))
        print(f"  GPS {len(gps)}개, hdop {gps[:,3].mean():.2f}")
        print(f"  정렬  yaw {math.degrees(yaw):+.2f}deg,  축척 {k:.4f},  잔차 RMS {rms:.2f} m")
        frame = "ENU"

        if mode == "warp":
            warp_t = gps[:, 0]
            warp_v = smooth(res, warp_t, args.sigma)
            mag = np.linalg.norm(warp_v, axis=1)
            print(f"  변형  sigma {args.sigma:.0f}s,  보정량 평균 {mag.mean():.2f} m "
                  f"max {mag.max():.2f} m")
    else:
        print(f"  고정 축척 k = {k}")

    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                   [math.sin(yaw), math.cos(yaw), 0.0],
                   [0.0, 0.0, 1.0]])

    def pose_at(i, t):
        """(회전3x3, 위치3) 반환 — 보정 적용 후"""
        R = Rz @ quat_to_R(odo[i, 4], odo[i, 5], odo[i, 6], odo[i, 7])
        p = odo[i, 1:4].copy()
        # 위치: 상사변환 (축척은 위치에만)
        xy = k * (Rz[:2, :2] @ p[:2]) + T
        p = np.array([xy[0], xy[1], p[2] * k])
        if warp_v is not None:
            p[0] += float(np.interp(t, warp_t, warp_v[:, 0]))
            p[1] += float(np.interp(t, warp_t, warp_v[:, 1]))
        return R, p

    # ---------- 점군 누적 ----------
    cid = topic_id(cur, CLOUD_TOPIC)
    if cid is None:
        print("  /utlidar/cloud 없음 — 건너뜀")
        con.close()
        return None
    rows = cur.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (cid,)
    ).fetchall()

    acc, used, skipped = [], 0, 0
    for n, (ts, data) in enumerate(rows):
        if n % args.stride:
            continue
        t = ts / 1e9
        i, dt = nearest_odom(odo, t)
        if dt > args.max_dt:
            skipped += 1
            continue
        pts = cloud_xyz(deserialize_message(data, get_message("sensor_msgs/msg/PointCloud2")))
        if pts.size == 0:
            continue
        d = np.linalg.norm(pts, axis=1)
        pts = pts[(d > args.min_range) & (d < args.max_range)]
        if pts.size == 0:
            continue
        pb = pts @ R_BL.T + lever
        R, p = pose_at(i, t)
        acc.append(pb @ R.T + p)
        used += 1
        if used % 500 == 0:
            print(f"    ... {used}", flush=True)

    con.close()
    if not acc:
        print("  누적 점 없음")
        return None

    Q = np.vstack(acc)
    Q = Q[(Q[:, 2] > args.z_min) & (Q[:, 2] < args.z_max)]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(Q)
    pcd = pcd.voxel_down_sample(args.voxel)
    P3 = np.asarray(pcd.points)
    lo, hi = P3.min(0), P3.max(0)

    print(f"  스캔 {used} (건너뜀 {skipped}),  복셀 후 {len(P3):,} 점  [{frame}]")
    print(f"  범위 x {lo[0]:.1f}~{hi[0]:.1f}  y {lo[1]:.1f}~{hi[1]:.1f}  z {lo[2]:.1f}~{hi[2]:.1f}")

    os.makedirs(OUTDIR, exist_ok=True)
    out = os.path.join(OUTDIR, f"{tag}.pcd")
    o3d.io.write_point_cloud(out, pcd)
    print(f"  저장 {out}")
    return out


def make_grid(pcd_path, res):
    """pcd_to_grid.py 를 pcd 별 폴더에서 돌려 결과가 안 덮이게 한다."""
    tag = os.path.splitext(os.path.basename(pcd_path))[0]
    d = os.path.join(OUTDIR, "grid_" + tag)
    os.makedirs(d, exist_ok=True)
    cmd = [sys.executable, GRID, pcd_path, "map"]
    if res is not None:
        cmd.append(str(res))
    try:
        r = subprocess.run(cmd, cwd=d, capture_output=True, text=True, timeout=1800)
    except Exception as e:
        print(f"  [격자 실패] {e}")
        return
    for ln in r.stdout.splitlines():
        if any(w in ln for w in ("격자", "장애물 칸", "해상도", "저장")):
            print("   " + ln.strip())
    if r.returncode != 0:
        print(f"  [격자 오류] {r.stderr.strip()[:300]}")
    else:
        print(f"   격자 -> {d}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags", nargs="*", default=None)
    ap.add_argument("--modes", nargs="*", default=["none", "rigid", "warp"],
                    choices=["none", "rigid", "warp"])
    ap.add_argument("--voxel", type=float, default=0.15)
    ap.add_argument("--grid-res", type=float, default=0.15,
                    help="격자 해상도 m.  pcd_to_grid 가 인자를 안 받으면 무시된다")
    ap.add_argument("--no-grid", action="store_true")
    ap.add_argument("--k", type=float, default=K_OUTDOOR, help="mode=none 의 고정 축척")
    ap.add_argument("--sigma", type=float, default=30.0, help="warp 평활 시정수 초")
    ap.add_argument("--min-range", type=float, default=0.6)
    ap.add_argument("--max-range", type=float, default=40.0)
    ap.add_argument("--z-min", type=float, default=-2.0)
    ap.add_argument("--z-max", type=float, default=8.0)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--max-dt", type=float, default=0.05)
    args = ap.parse_args()

    if args.bags:
        bags = [os.path.join(BAGDIR, b) if not os.path.isabs(b) else b for b in args.bags]
    else:
        bags = sorted(d for d in glob.glob(os.path.join(BAGDIR, "go2_*"))
                      if glob.glob(os.path.join(d, "*.db3")))
    if not bags:
        sys.exit(f"bag 을 찾지 못했습니다: {BAGDIR}")

    print("대상 bag:")
    for b in bags:
        print("  " + os.path.basename(b))
    print("방식:", ", ".join(args.modes))

    made = []
    for b in bags:
        for m in args.modes:
            p = build(b, m, args)
            if p:
                made.append(p)
                if not args.no_grid:
                    make_grid(p, args.grid_res)

    print(f"\n{'='*62}\n완료: pcd {len(made)}개")
    for p in made:
        print("  " + p)
    print(f"\n격자 결과는 {OUTDIR}/grid_<이름>/ 안의 map.pgm / map.yaml")
    print("픽셀당 실제 거리는 map.yaml 의 resolution 항목이 정답입니다.")


if __name__ == "__main__":
    main()
