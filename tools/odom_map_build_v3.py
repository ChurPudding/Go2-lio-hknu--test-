#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_map_build_v3.py — /utlidar/cloud (raw) 로 점군을 누적한다

v2 와의 차이
------------
v2 는 /utlidar/cloud_deskewed 를 쓴다. 그 토픽의 점은 이미 odom 좌표계라
좌표 변환도 레버암도 필요 없다.

실외 bag(0812)에는 cloud_deskewed 가 녹화되지 않았고 /utlidar/cloud 만 있다.
이 토픽의 점은 **라이다(radar) 프레임** 이므로 두 단계를 직접 해줘야 한다.

    p_body = R_BL · p_lidar + LEVER          (go2_calib.py 의 실측 외부파라미터)
    p_odom = R_odom(t) · p_body + t_odom(t)  (/utlidar/robot_odom)

축척 보정은 v2 와 동일하다. 오차는 로봇 위치 t(t) 에만 있으므로

    p'(t) = p_odom(t) + (k-1)·t_odom(t)

로 각 프레임을 밀어주면 궤적만 k 배가 되고 스캔 자체는 그대로 남는다.

k 의 근거
---------
  실내 줄자 기준 (2026-08-10)        k = 1.1995
  실외 GPS 정렬 loop1_1440 (08-12)   k = 1.1910
  실외 GPS 정렬 loop1_1449 (08-12)   k = 1.2327
  → 서로 독립적인 세 방법이 1.19~1.23 으로 일치. 기본값 1.1995 를 쓴다.

주의
----
raw cloud 는 모션 보정(deskew)이 안 되어 있다. 회전이 빠르면 스캔이 밀린다.
0812 폐루프는 제자리 회전을 피하고 호를 그리며 돌았으므로 영향이 작을 것으로
본다. 결과가 흐릿하면 --max-omega 로 빠른 회전 구간을 버려볼 것.

사용
----
  python3 odom_map_build_v3.py <bag> [voxel] [출력.pcd] [옵션]

  # 먼저 짧은 bag 으로 파라미터 확인 (46초, 1분이면 끝난다)
  python3 odom_map_build_v3.py ~/data/bags/outdoor/0812/go2_slope_0812_1500 \\
      0.15 ~/fastlio_ws/results/outdoor_0812/slope_test.pcd

  # 본편
  python3 odom_map_build_v3.py ~/data/bags/outdoor/0812/go2_loop1_0812_1449 \\
      0.15 ~/fastlio_ws/results/outdoor_0812/loop1_1449.pcd
"""

import argparse
import glob
import math
import os
import sqlite3
import sys

import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d 가 없습니다.")

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2

PC2 = get_message("sensor_msgs/msg/PointCloud2")
ODOM = get_message("nav_msgs/msg/Odometry")

CLOUD_TOPIC = "/utlidar/cloud"
ODOM_TOPIC = "/utlidar/robot_odom"


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from go2_calib import K_OUTDOOR

def cloud_xyz(msg):
    """PointCloud2 -> (N,3) float64.  필드 datatype 이 섞여 있어도 동작한다."""
    off = {f.name: (f.offset, f.datatype) for f in msg.fields}
    for nm in ("x", "y", "z"):
        if nm not in off:
            return np.empty((0, 3))
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3))
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    raw = raw[: n * msg.point_step].reshape(n, msg.point_step)
    cols = []
    for nm in ("x", "y", "z"):
        o, dt = off[nm]
        if dt == 7:      # FLOAT32
            c = raw[:, o:o + 4].copy().view(np.float32).ravel()
        elif dt == 8:    # FLOAT64
            c = raw[:, o:o + 8].copy().view(np.float64).ravel()
        else:
            raise RuntimeError(f"{nm} datatype {dt} 미지원")
        cols.append(c.astype(np.float64))
    P = np.column_stack(cols)
    return P[np.isfinite(P).all(axis=1)]



# ----------------------------------------------------------------------
# 외부 파라미터
# ----------------------------------------------------------------------
def get_extrinsic(invert):
    """(R_BL, LEVER) 반환.  R_BL 은 lidar->body 회전."""
    sys.path.insert(0, os.path.expanduser("~/fastlio_ws/tools"))
    try:
        from go2_calib import LEVER, R_LB
    except Exception as e:
        sys.exit(f"go2_calib.py 를 불러오지 못했습니다: {e}")

    R_LB = np.asarray(R_LB, dtype=float).reshape(3, 3)
    lever = np.asarray(LEVER, dtype=float).reshape(3)

    # go2_calib 의 R_LB 는 acc_lidar = R_LB @ acc_body, 즉 body->lidar.
    # 점을 lidar->body 로 옮기려면 전치가 필요하다.
    R_BL = R_LB.T
    if invert:
        R_BL = R_LB
    return R_BL, lever


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ----------------------------------------------------------------------
# bag 읽기
# ----------------------------------------------------------------------
def open_bag(bag_dir):
    db = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    if not db:
        sys.exit(f"db3 를 찾지 못했습니다: {bag_dir}")
    return sqlite3.connect(db[0])


def topic_id(cur, name):
    r = cur.execute("SELECT id FROM topics WHERE name=?", (name,)).fetchone()
    return r[0] if r else None


def load_odom(cur):
    """[t, x,y,z, qx,qy,qz,qw] 배열"""
    tid = topic_id(cur, ODOM_TOPIC)
    if tid is None:
        sys.exit(f"{ODOM_TOPIC} 가 bag 에 없습니다.")
    rows = cur.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
    ).fetchall()
    out = []
    for ts, data in rows:
        m = deserialize_message(data, ODOM)
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        out.append((ts / 1e9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return np.array(out)


def nearest_odom(odo, t):
    i = int(np.searchsorted(odo[:, 0], t))
    if i <= 0:
        i = 0
    elif i >= len(odo):
        i = len(odo) - 1
    elif abs(odo[i - 1, 0] - t) < abs(odo[i, 0] - t):
        i -= 1
    return i, abs(odo[i, 0] - t)


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("voxel", nargs="?", type=float, default=0.15,
                    help="복셀 크기 m (실외 권장 0.15~0.25, 실내는 0.05)")
    ap.add_argument("out", nargs="?",
                    default=os.path.expanduser("~/fastlio_ws/results/outdoor_0812/scans.pcd"))
    ap.add_argument("--k", type=float, default=K_OUTDOOR, help="축척 보정 계수 (기본 K_OUTDOOR)")
    ap.add_argument("--min-range", type=float, default=0.6,
                    help="이보다 가까운 점 제거 (로봇 몸통)")
    ap.add_argument("--max-range", type=float, default=40.0,
                    help="이보다 먼 점 제거. 실외는 넉넉히")
    ap.add_argument("--z-min", type=float, default=-2.0)
    ap.add_argument("--z-max", type=float, default=8.0)
    ap.add_argument("--stride", type=int, default=1,
                    help="N 프레임마다 1개만 사용 (빠른 시험용)")
    ap.add_argument("--max-dt", type=float, default=0.05,
                    help="odom 과 시각차가 이보다 크면 그 스캔은 버림")
    ap.add_argument("--invert-extrinsic", action="store_true",
                    help="R_BL 방향이 반대일 때 사용")
    ap.add_argument("--no-scale", action="store_true", help="축척 보정 끄기")
    ap.add_argument("--elev", action="store_true",
                    help="피치 적분 고도를 z 에 넣는다. 다리 오도메트리의 "
                         "position[2] 는 고도가 아니라 몸통 높이이므로 "
                         "경사지에서 지도가 평면으로 눌리는 것을 막는다")
    args = ap.parse_args()

    k = 1.0 if args.no_scale else args.k
    R_BL, lever = get_extrinsic(args.invert_extrinsic)

    print(f"bag        : {args.bag}")
    print(f"voxel      : {args.voxel} m")
    print(f"k          : {k}")
    print(f"range      : {args.min_range} ~ {args.max_range} m")
    print(f"R_BL det   : {np.linalg.det(R_BL):+.4f}   LEVER {lever}")

    con = open_bag(args.bag)
    cur = con.cursor()
    odo = load_odom(cur)
    print(f"odom       : {len(odo)} msgs, {odo[-1,0]-odo[0,0]:.1f} s")

    elev = None
    if args.elev:
        import sqlite3 as _sq
        SMS = get_message("unitree_go/msg/SportModeState")
        _db = glob.glob(os.path.join(args.bag, "*.db3"))[0]
        _c = _sq.connect(_db); _cu = _c.cursor()
        _r = _cu.execute("SELECT id FROM topics WHERE name='/sportmodestate'").fetchone()
        E = []
        for _ts, _d in _cu.execute(
                "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (_r[0],)):
            _m = deserialize_message(_d, SMS)
            E.append((_ts/1e9, _m.position[0], _m.position[1], _m.imu_state.rpy[1]))
        _c.close()
        E = np.array(E)
        _ds = np.linalg.norm(np.diff(E[:, 1:3], axis=0), axis=1) * k
        _pm = 0.5 * (E[1:, 3] + E[:-1, 3])
        _z = np.concatenate([[0.0], np.cumsum(-np.sin(_pm) * _ds)])
        elev = (E[:, 0], _z)
        print(f"고도 보정   : Δz {_z[-1]:+.2f} m  (범위 {_z.min():+.2f}~{_z.max():+.2f})")

    cid = topic_id(cur, CLOUD_TOPIC)
    if cid is None:
        sys.exit(f"{CLOUD_TOPIC} 가 bag 에 없습니다.")
    rows = cur.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (cid,)
    ).fetchall()
    print(f"cloud      : {len(rows)} scans")

    acc = []
    used = skipped = 0
    for n, (ts, data) in enumerate(rows):
        if n % args.stride:
            continue
        t = ts / 1e9
        i, dt = nearest_odom(odo, t)
        if dt > args.max_dt:
            skipped += 1
            continue

        msg = deserialize_message(data, PC2)
        pts = cloud_xyz(msg)
        if pts.size == 0:
            continue
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)

        # 거리 필터 (라이다 프레임에서)
        d = np.linalg.norm(pts, axis=1)
        pts = pts[(d > args.min_range) & (d < args.max_range)]
        if pts.size == 0:
            continue

        # lidar -> body
        pb = pts @ R_BL.T + lever

        # body -> odom  (+ 축척 보정)
        R = quat_to_R(odo[i, 4], odo[i, 5], odo[i, 6], odo[i, 7])
        tv = odo[i, 1:4]
        tv2 = tv + (k - 1.0) * tv
        if elev is not None:
            tv2 = tv2.copy()
            tv2[2] += float(np.interp(t, elev[0], elev[1]))
        po = pb @ R.T + tv2

        acc.append(po)
        used += 1

        if used % 200 == 0:
            print(f"  ... {used} scans", flush=True)

    con.close()
    if not acc:
        sys.exit("누적된 점이 없습니다. --max-dt 나 range 를 확인하십시오.")

    P = np.vstack(acc)
    print(f"\n사용 {used} scans (건너뜀 {skipped}), 원본 점 {len(P):,}")

    m = (P[:, 2] > args.z_min) & (P[:, 2] < args.z_max)
    P = P[m]
    print(f"z 필터 후 {len(P):,}")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(P)
    pcd = pcd.voxel_down_sample(args.voxel)
    print(f"복셀 후   {len(pcd.points):,}")

    lo = np.min(np.asarray(pcd.points), axis=0)
    hi = np.max(np.asarray(pcd.points), axis=0)
    print(f"범위       x {lo[0]:.1f}~{hi[0]:.1f}  "
          f"y {lo[1]:.1f}~{hi[1]:.1f}  z {lo[2]:.1f}~{hi[2]:.1f}")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    o3d.io.write_point_cloud(args.out, pcd)
    print(f"\n저장: {args.out}")
    print("보기: python3 ~/fastlio_ws/tools/pcd_view.py " + args.out)


if __name__ == "__main__":
    main()
