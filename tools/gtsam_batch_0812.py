#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gtsam_batch_0812.py — 오프라인 배치 팩터그래프 최적화 (1단계)

무엇을 하는가
-------------
다리 오도메트리(상대)와 GPS(절대)를 팩터 그래프로 묶어 자세 궤적을 푼다.
build_maps_0812.py 의 rigid/warp 를 원리적인 방법으로 대체하는 단계다.

  rigid  전역 상사변환 1회        — 사람이 정한 규칙
  warp   잔차를 sigma=30s 로 평활 — 평활 세기를 사람이 지정
  이것   두 센서의 신뢰도(sigma)로 최적점을 계산

그래프 구성
-----------
  변수   Pose2(x, y, theta)  — GPS 시각마다 하나 (1 Hz, 약 320개)
         3D 가 아닌 이유: 다리 오도메트리 z 는 고도가 아니라 몸통 높이이고
         (2026-08-12 실측 dz≈0), GPS 고도는 수평보다 2~3배 나쁘다.
         관측이 없는 변수를 추정하면 최적화가 불안정해진다.

  팩터   BetweenFactorPose2   연속 자세 사이. 오도메트리 적분값. sigma 작음
         PriorFactorPose2     GPS 위치. theta sigma 를 크게 줘서 x,y 만 구속
                              (GTSAM 의 GPSFactor 는 Pose3 전용이라 이 방식을 쓴다)
         PriorFactorPose2     첫 자세. 상사변환 초기추정에서 가져옴

  로버스트  GPS 팩터에 Huber(1.345). 0812 정지 실측에서 GPS 오차가
            연속 표류가 아니라 계단형 점프(2회)로 나타났기 때문이다.
            가우시안 sigma 하나로는 점프 순간의 팩터가 전체를 끌고 간다.

  hdop 가중  sigma_i = base * hdop_i / hdop_median
             위성 수는 통제할 수 없으므로 조건을 기록해 대응한다.

축척 k
------
  BetweenFactor 의 평행이동에만 곱한다(회전은 참값). k 를 바꿔가며 그래프를
  다시 풀어 GPS 잔차가 최소인 지점을 찾는다.

  현재까지: 실내 줄자 1.1995 / GPS 정렬 1.1910(1440) / 1.2327(1449)

초기 추정
---------
  상사변환(yaw+축척+평행이동)으로 오도메트리를 GPS ENU 에 맞춘 값을 쓴다.
  비선형 최적화는 초기값이 나쁘면 국소최소로 빠진다. 특히 절대 방위는
  오도메트리에 없는 정보라 반드시 정렬로 잡아줘야 한다.

사용
----
  python3 gtsam_batch_0812.py                      # loop1_1449, k 스윕
  python3 gtsam_batch_0812.py --bag <경로>
  python3 gtsam_batch_0812.py --k 1.1995 --no-sweep
  python3 gtsam_batch_0812.py --plot
"""

import argparse
import glob
import json
import math
import os
import sqlite3
import sys

import numpy as np

KEEP = None

try:
    import gtsam
except ImportError:
    sys.exit("gtsam 이 없습니다:  pip3 install gtsam --break-system-packages")

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ODOM = get_message("nav_msgs/msg/Odometry")
STR = get_message("std_msgs/msg/String")

DEFAULT_BAG = os.path.expanduser(
    "~/data/bags/outdoor/0812/go2_loop1_0812_1449")
OUTDIR = os.path.expanduser("~/fastlio_ws/results/outdoor_0812/gtsam")


# ----------------------------------------------------------------------
# 읽기
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


def load(bag_dir):
    db = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    if not db:
        sys.exit(f"db3 없음: {bag_dir}")
    con = sqlite3.connect(db[0])
    cur = con.cursor()

    def grab(topic):
        r = cur.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
        if not r:
            return []
        return cur.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (r[0],)).fetchall()

    odo = []
    for ts, data in grab("/utlidar/robot_odom"):
        m = deserialize_message(data, ODOM)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        odo.append((ts / 1e9, p.x, p.y, yaw))

    gps = []
    for ts, data in grab("/gnss"):
        try:
            j = json.loads(deserialize_message(data, STR).data)
        except Exception:
            continue
        if j.get("fixed") != 1:
            continue
        gps.append((ts / 1e9, j["latitude"], j["longitude"],
                    float(j.get("hdop", 1.0) or 1.0)))

    con.close()
    if not odo:
        sys.exit("/utlidar/robot_odom 이 없습니다.")
    if len(gps) < 20:
        sys.exit(f"GPS fix 가 {len(gps)}개뿐입니다. 이 bag 은 배치 최적화에 못 씁니다.")
    return np.array(odo), np.array(gps)


def to_enu(gps):
    lat0, lon0 = gps[0, 1], gps[0, 2]
    _MLAT, _MLON = _m_per_deg(lat0)
    e = (gps[:, 2] - lon0) * _MLON
    n = (gps[:, 1] - lat0) * _MLAT
    return np.column_stack([e, n]), lat0, lon0


def similarity_2d(src, dst):
    """src -> dst 최적 상사변환. (yaw_rad, scale, t(2,))"""
    sc, dc = src.mean(0), dst.mean(0)
    a, b = src - sc, dst - dc
    u, sv, vt = np.linalg.svd(a.T @ b)
    R = vt.T @ u.T
    if np.linalg.det(R) < 0:
        vt[-1] *= -1
        R = vt.T @ u.T
    var = (a ** 2).sum()
    s = sv.sum() / var if var > 1e-9 else 1.0
    return math.atan2(R[1, 0], R[0, 0]), float(s), dc - s * (R @ sc)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ----------------------------------------------------------------------
# 그래프
# ----------------------------------------------------------------------
def build_and_solve(nodes, P, hdop, k, args, verbose=False):
    """nodes: (N,3) 각 GPS 시각의 오도메트리 자세 (x,y,yaw)  [odom 프레임]
       P:     (N,2) 같은 시각의 GPS ENU
       반환:  (N,3) 최적화된 자세 [ENU 프레임]"""
    n = len(nodes)

    # --- 초기 추정: 상사변환 정렬 ---
    yaw0, _, t0 = similarity_2d(nodes[:, :2], P)
    c, s = math.cos(yaw0), math.sin(yaw0)
    Rz = np.array([[c, -s], [s, c]])
    init_xy = k * (nodes[:, :2] @ Rz.T) + t0
    init_th = nodes[:, 2] + yaw0

    global KEEP
    if KEEP is None or len(KEEP) != n:
        KEEP = np.ones(n, bool)
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    # --- 노이즈 모델 ---
    # 첫 자세: GPS 수준으로 느슨하게. 방위는 정렬값을 어느 정도 믿는다.
    prior_n = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([args.gps_sigma, args.gps_sigma, math.radians(10.0)]))

    hd_med = float(np.median(hdop)) if np.median(hdop) > 0 else 1.0

    graph.add(gtsam.PriorFactorPose2(
        0, gtsam.Pose2(init_xy[0, 0], init_xy[0, 1], init_th[0]), prior_n))
    initial.insert(0, gtsam.Pose2(init_xy[0, 0], init_xy[0, 1], init_th[0]))

    for i in range(1, n):
        initial.insert(i, gtsam.Pose2(init_xy[i, 0], init_xy[i, 1], init_th[i]))

        # --- 오도메트리 BetweenFactor ---
        # odom 프레임에서의 상대 변환. 평행이동에만 k 를 곱한다.
        dth = wrap(nodes[i, 2] - nodes[i - 1, 2])
        d = nodes[i, :2] - nodes[i - 1, :2]
        cp, sp = math.cos(-nodes[i - 1, 2]), math.sin(-nodes[i - 1, 2])
        dx = k * (cp * d[0] - sp * d[1])
        dy = k * (sp * d[0] + cp * d[1])
        dist = math.hypot(dx, dy)

        # 상대 오차는 거리에 비례. 0812 폐루프 0.31%/277m 를 근거로
        # 보수적으로 1% + 하한 2cm.
        sxy = args.odom_rel * dist + 0.02
        sth = args.odom_yaw_rate * max(nodes[i, 0] - nodes[i - 1, 0], 1e-3) \
            + math.radians(0.05)
        odo_n = gtsam.noiseModel.Diagonal.Sigmas(np.array([sxy, sxy, sth]))
        graph.add(gtsam.BetweenFactorPose2(i - 1, i,
                                           gtsam.Pose2(dx, dy, dth), odo_n))

    # --- GPS 팩터 ---
    # GTSAM 의 GPSFactor 는 Pose3 전용이므로, theta sigma 를 아주 크게 준
    # PriorFactorPose2 로 x,y 만 구속한다.
    for i in range(n):
        if not KEEP[i]:
            continue
        sg = args.gps_sigma * (hdop[i] / hd_med)
        base = gtsam.noiseModel.Diagonal.Sigmas(np.array([sg, sg, 1e6]))
        if args.huber > 0:
            model = gtsam.noiseModel.Robust.Create(
                gtsam.noiseModel.mEstimator.Huber.Create(args.huber), base)
        else:
            model = base
        graph.add(gtsam.PriorFactorPose2(
            i, gtsam.Pose2(P[i, 0], P[i, 1], 0.0), model))

    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(args.iters)
    if verbose:
        params.setVerbosityLM("SUMMARY")
    opt = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = opt.optimize()

    out = np.empty((n, 3))
    for i in range(n):
        p = result.atPose2(i)
        out[i] = (p.x(), p.y(), p.theta())
    return out, float(graph.error(result)), float(graph.error(initial))


def report(name, X, P):
    res = np.linalg.norm(X[:, :2] - P, axis=1)
    L = np.linalg.norm(np.diff(X[:, :2], axis=0), axis=1).sum()
    close = float(np.linalg.norm(X[-1, :2] - X[0, :2]))
    dyaw = math.degrees(wrap(X[-1, 2] - X[0, 2]))
    print(f"  {name:<12} 잔차 RMS {np.sqrt((res**2).mean()):6.2f} m  "
          f"max {res.max():5.2f} m  |  경로장 {L:6.1f} m  "
          f"폐루프 {close:5.2f} m ({close/L*100:.2f}%)  yaw {dyaw:+.2f}deg")
    return float(np.sqrt((res ** 2).mean())), L, close


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default=DEFAULT_BAG)
    ap.add_argument("--gps-sigma", type=float, default=5.0,
                    help="GPS factor 기준 sigma (m). 0812 정지 실측 근거 5 m")
    ap.add_argument("--huber", type=float, default=1.345,
                    help="Huber k. 0 이면 로버스트 끔")
    ap.add_argument("--odom-rel", type=float, default=0.01,
                    help="오도메트리 상대오차 비율 (거리당)")
    ap.add_argument("--odom-yaw-rate", type=float, default=math.radians(0.02),
                    help="yaw 드리프트 rad/s")
    ap.add_argument("--k", type=float, default=K_OUTDOOR)
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--sweep-lo", type=float, default=1.10)
    ap.add_argument("--sweep-hi", type=float, default=1.35)
    ap.add_argument("--sweep-n", type=int, default=26)
    ap.add_argument("--iters", type=int, default=100)
    ap.add_argument("--drop", nargs=2, type=float, default=None,
                    metavar=("T0", "T1"),
                    help="이 시간대(초)의 GPS 를 버린다. 음영 구간 모의")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    name = os.path.basename(args.bag)
    print(f"bag: {name}")
    odo, gps = load(args.bag)
    P, lat0, lon0 = to_enu(gps)
    hdop = gps[:, 3]
    keep = np.ones(len(gps), bool)
    if args.drop:
        rel = gps[:, 0] - gps[0, 0]
        keep = ~((rel >= args.drop[0]) & (rel <= args.drop[1]))
        print(f"  GPS 음영 모의: {args.drop[0]:.0f}~{args.drop[1]:.0f}s "
              f"의 {int((~keep).sum())}개 제거")
    print(f"  odom {len(odo)} msgs, gps fix {len(gps)}개, "
          f"hdop 중앙값 {np.median(hdop):.2f}")

    # GPS 시각에 오도메트리 보간 -> 자세 노드
    t = gps[:, 0]
    nodes = np.column_stack([
        np.interp(t, odo[:, 0], odo[:, 1]),
        np.interp(t, odo[:, 0], odo[:, 2]),
        np.unwrap(np.interp(t, odo[:, 0], np.unwrap(odo[:, 3]))),
    ])
    globals()['KEEP'] = keep
    print(f"  자세 노드 {len(nodes)}개 (GPS 시각 기준)")

    # --- 비교 기준: 상사변환만 (= rigid) ---
    yaw_r, k_r, t_r = similarity_2d(nodes[:, :2], P)
    c, s = math.cos(yaw_r), math.sin(yaw_r)
    Rz = np.array([[c, -s], [s, c]])
    rigid = np.column_stack([k_r * (nodes[:, :2] @ Rz.T) + t_r,
                             nodes[:, 2] + yaw_r])
    print(f"\n[기준] rigid 상사변환  축척 {k_r:.4f}, yaw {math.degrees(yaw_r):+.2f}deg")
    report("rigid", rigid, P)

    # --- k 스윕 ---
    best = None
    if not args.no_sweep:
        print(f"\n[k 스윕] {args.sweep_lo}~{args.sweep_hi}, {args.sweep_n}점")
        rows = []
        for k in np.linspace(args.sweep_lo, args.sweep_hi, args.sweep_n):
            X, err, _ = build_and_solve(nodes, P, hdop, float(k), args)
            r = np.linalg.norm(X[:, :2] - P, axis=1)
            rms = float(np.sqrt((r ** 2).mean()))
            L = np.linalg.norm(np.diff(X[:, :2], axis=0), axis=1).sum()
            rows.append((float(k), rms, err, L))
            if best is None or err < best[2]:
                best = (float(k), rms, err, X)
        print(f"  {'k':>7} {'잔차RMS':>9} {'그래프오차':>12} {'경로장':>8}")
        for k, rms, err, L in rows:
            mark = " <-" if abs(k - best[0]) < 1e-9 else ""
            print(f"  {k:7.4f} {rms:9.3f} {err:12.1f} {L:8.1f}{mark}")
        print(f"\n  ** 최적 k = {best[0]:.4f} **")
        print(f"     실내 줄자 1.1995 / GPS정렬 1.1910·1.2327 과 비교하십시오")
        k_use, X = best[0], best[3]
    else:
        k_use = args.k
        X, err, err0 = build_and_solve(nodes, P, hdop, k_use, args, verbose=True)
        print(f"\n  그래프 오차 {err0:.1f} -> {err:.1f}")

    print(f"\n[결과] k = {k_use:.4f}, GPS sigma {args.gps_sigma} m, "
          f"Huber {args.huber}")
    rms_g, L, close = report("gtsam", X, P)
    if args.drop:
        rel = t - t[0]
        w = (rel >= args.drop[0]) & (rel <= args.drop[1])
        if w.any():
            rw = np.linalg.norm(X[w, :2] - P[w], axis=1)
            seg = np.linalg.norm(np.diff(X[w, :2], axis=0), axis=1).sum()
            print(f"\n  [음영구간 {args.drop[0]:.0f}~{args.drop[1]:.0f}s]"
                  f"  주행 {seg:.1f} m"
                  f"  잔차 평균 {rw.mean():.2f} m  max {rw.max():.2f} m")
    rms_r, _, _ = report("rigid", rigid, P)
    print(f"\n  잔차 RMS  rigid {rms_r:.2f} m  ->  gtsam {rms_g:.2f} m  "
          f"({(1-rms_g/rms_r)*100:+.1f}%)")
    print("  주의: GPS 잔차만으로 판정하면 안 됩니다. sigma 를 줄이면 잔차는")
    print("        항상 내려가지만 궤적이 GPS 노이즈를 그대로 따라갑니다.")
    print("        폐루프 오차·yaw 와 함께 보십시오.")

    # --- 저장 ---
    os.makedirs(OUTDIR, exist_ok=True)
    out = os.path.join(OUTDIR, f"{name}_poses.npz")
    np.savez(out, t=t, xytheta=X, gps_enu=P, hdop=hdop,
             lat0=lat0, lon0=lon0, k=k_use,
             gps_sigma=args.gps_sigma, huber=args.huber)
    print(f"\n저장: {out}")
    print("  이 자세를 점군 누적에 쓰려면 build_maps 에 mode=gtsam 을 추가하면 됩니다.")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            return
        fig, ax = plt.subplots(1, 2, figsize=(14, 6.5))
        ax[0].plot(P[:, 0], P[:, 1], ".", ms=3, color="tab:orange",
                   alpha=0.6, label="GPS (ENU)")
        ax[0].plot(rigid[:, 0], rigid[:, 1], "-", lw=1.2,
                   color="tab:gray", label="rigid")
        ax[0].plot(X[:, 0], X[:, 1], "-", lw=1.8,
                   color="tab:blue", label="GTSAM")
        ax[0].plot(X[0, 0], X[0, 1], "o", ms=9, color="green")
        ax[0].plot(X[-1, 0], X[-1, 1], "s", ms=9, color="red")
        ax[0].set_aspect("equal")
        ax[0].grid(alpha=0.3, ls=":")
        ax[0].set_xlabel("East [m]")
        ax[0].set_ylabel("North [m]")
        ax[0].legend(fontsize=9)
        ax[0].set_title(f"{name}  k={k_use:.4f}")

        tt = t - t[0]
        ax[1].plot(tt, np.linalg.norm(rigid[:, :2] - P, axis=1),
                   lw=1, color="tab:gray", label="rigid")
        ax[1].plot(tt, np.linalg.norm(X[:, :2] - P, axis=1),
                   lw=1.4, color="tab:blue", label="GTSAM")
        ax[1].axhline(args.gps_sigma, ls="--", lw=1, color="crimson")
        ax[1].grid(alpha=0.3, ls=":")
        ax[1].set_xlabel("t [s]")
        ax[1].set_ylabel("GPS residual [m]")
        ax[1].legend(fontsize=9)
        ax[1].set_title("residual vs time (dashed = sigma)")

        fig.tight_layout()
        p = os.path.join(OUTDIR, f"{name}_gtsam.png")
        fig.savefig(p, dpi=140)
        print(f"저장: {p}")


if __name__ == "__main__":
    main()
