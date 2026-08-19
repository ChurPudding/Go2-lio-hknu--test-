#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[python3 로 실행 — source 대상 아님]

odom_scale_check.py — /utlidar/robot_odom 과 /lf/sportmodestate 의 스케일 검증

무엇을 답하는가
--------------
  (1) /utlidar/robot_odom 이 /lf/sportmodestate 와 같은 값인가, 다른 값인가
      -> 같다면 A 쪽에 스케일 보정 노드 하나만 끼우면 끝난다.
      -> 다르다면 두 소스의 k 를 각각 재야 한다.
  (2) 실측 거리 대비 k 는 얼마인가
      -> 5m 직진 bag + 줄자 실측값을 --truth 로 넣으면 k = 실측/측정 이 나온다.

왜 chord 로 재는가
-----------------
경로장(path length)은 보행 진동이 그대로 더해져 항상 과대평가된다. 직진 검증은
시작점-끝점 직선거리(chord)로 재야 한다. 시작·끝의 정지 구간 평균을 쓰면
보행 흔들림이 평균에서 지워져 재현성이 올라간다.

전제
----
  - 로봇이 정지 -> 직진 -> 정지 하는 bag
  - 시작과 끝에 각 2초 이상 정지 구간이 있을 것 (없으면 --settle 0 으로 끔)
  - 줄자로 실제 이동거리를 재 둘 것

사용
----
    srcoff
    python3 odom_scale_check.py <bag_dir> --truth 5.00
    python3 odom_scale_check.py <bag_dir> --truth 5.00 --plot
    python3 odom_scale_check.py <bag_dir>            # k 없이 두 소스 비교만

    # 3회 반복 bag 을 한 번에
    for b in ~/data/bags/scale/run*; do
        python3 odom_scale_check.py "$b" --truth 5.00
    done
"""

import argparse
import math
import os
import sys

import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


ODOM_TOPIC = "/utlidar/robot_odom"
SPORT_TOPIC = "/lf/sportmodestate"
SPORT_FALLBACK = "/sportmodestate"

MOVE_SPEED = 0.08      # m/s. 이 위를 '이동 중' 으로 본다
SETTLE = 1.5           # s. 시작·끝 정지 구간에서 평균낼 길이


# ----------------------------------------------------------------------
def read_bag(bag, want):
    """want 에 있는 토픽만 읽어 {토픽: [(t, msg), ...]} 로 돌려준다."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    have = [t for t in want if t in types]
    if not have:
        print(f"[FAIL] {want} 중 어느 것도 없습니다. 녹화된 토픽:")
        for name in sorted(types):
            print(f"        {name}   ({types[name]})")
        sys.exit(1)

    cls = {t: get_message(types[t]) for t in have}
    reader.set_filter(rosbag2_py.StorageFilter(topics=have))

    out = {t: [] for t in have}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        out[topic].append((t_ns / 1e9, deserialize_message(raw, cls[topic])))
    return out, types


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def unpack_sport(rows):
    """SportModeState -> t, xy, yaw, speed"""
    t, xy, yaw, spd = [], [], [], []
    for ts, m in rows:
        t.append(ts)
        xy.append([m.position[0], m.position[1]])
        yaw.append(m.imu_state.rpy[2])
        spd.append(math.hypot(m.velocity[0], m.velocity[1]))
    return (np.array(t), np.array(xy), np.array(yaw), np.array(spd))


def unpack_odom(rows):
    """nav_msgs/Odometry -> t, xy, yaw, speed"""
    t, xy, yaw, spd = [], [], [], []
    for ts, m in rows:
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        v = m.twist.twist.linear
        t.append(ts)
        xy.append([p.x, p.y])
        yaw.append(yaw_from_quat(q.x, q.y, q.z, q.w))
        spd.append(math.hypot(v.x, v.y))
    return (np.array(t), np.array(xy), np.array(yaw), np.array(spd))


# ----------------------------------------------------------------------
def move_window(t, spd, thr=MOVE_SPEED):
    """이동 구간 [t_start, t_end] 를 찾는다."""
    m = spd > thr
    if m.sum() < 5:
        return None
    idx = np.where(m)[0]
    return t[idx[0]], t[idx[-1]]


def anchor(t, xy, t_ref, side, settle):
    """t_ref 앞(side=-1) 또는 뒤(side=+1) 의 정지 구간 평균 위치."""
    if settle <= 0:
        i = int(np.argmin(np.abs(t - t_ref)))
        return xy[i], 1
    if side < 0:
        m = (t >= t_ref - settle - 0.1) & (t <= t_ref - 0.1)
    else:
        m = (t >= t_ref + 0.1) & (t <= t_ref + settle + 0.1)
    if m.sum() < 3:                      # 정지 구간이 짧으면 최근접 샘플
        i = int(np.argmin(np.abs(t - t_ref)))
        return xy[i], 1
    return xy[m].mean(0), int(m.sum())


def measure(name, t, xy, spd, settle, thr):
    """한 소스의 chord / path 를 잰다."""
    win = move_window(t, spd, thr)
    if win is None:
        print(f"  [{name}] 이동 구간을 못 찾았습니다 (최대 속력 "
              f"{spd.max():.3f} m/s). 정지 bag 인지 확인해 주세요.")
        return None

    t0, t1 = win
    p0, n0 = anchor(t, xy, t0, -1, settle)
    p1, n1 = anchor(t, xy, t1, +1, settle)

    seg = (t >= t0) & (t <= t1)
    path = float(np.linalg.norm(np.diff(xy[seg], axis=0), axis=1).sum())
    chord = float(np.linalg.norm(p1 - p0))

    return {
        "t0": t0, "t1": t1, "dur": t1 - t0,
        "p0": p0, "p1": p1, "n0": n0, "n1": n1,
        "chord": chord, "path": path,
        "hz": len(t) / max(t[-1] - t[0], 1e-6),
        "n": len(t),
    }


def fit_rigid(src, dst, with_scale):
    """src -> dst. yaw + 평행이동 (+선택적 스케일). gps_align_0812.py 와 동일 방식."""
    sc, dc = src.mean(0), dst.mean(0)
    A, B = src - sc, dst - dc
    H = A.T @ B
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    s = float(S.sum() / (A ** 2).sum()) if with_scale else 1.0
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    res = dst - (s * (src - sc) @ R.T + dc)
    return yaw, s, np.linalg.norm(res, axis=1)


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--truth", type=float, default=None,
                    help="줄자로 잰 실제 직진 거리 [m]")
    ap.add_argument("--settle", type=float, default=SETTLE,
                    help="시작·끝 정지 구간 평균 길이 [s]. 0 이면 끔")
    ap.add_argument("--speed", type=float, default=MOVE_SPEED,
                    help="이동 판정 속력 문턱 [m/s]")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    data, types = read_bag(args.bag, [ODOM_TOPIC, SPORT_TOPIC, SPORT_FALLBACK])

    src = {}
    if SPORT_TOPIC in data and data[SPORT_TOPIC]:
        src["sport"] = (SPORT_TOPIC,) + unpack_sport(data[SPORT_TOPIC])
    elif SPORT_FALLBACK in data and data[SPORT_FALLBACK]:
        src["sport"] = (SPORT_FALLBACK,) + unpack_sport(data[SPORT_FALLBACK])
    if ODOM_TOPIC in data and data[ODOM_TOPIC]:
        src["odom"] = (ODOM_TOPIC,) + unpack_odom(data[ODOM_TOPIC])

    print(f"bag: {os.path.basename(os.path.normpath(args.bag))}")
    print()
    print("=== 스트림 ===")
    for key, (name, t, xy, yaw, spd) in src.items():
        print(f"  {name:<26} {len(t):>6} msg  {t[-1]-t[0]:6.1f} s  "
              f"{len(t)/max(t[-1]-t[0],1e-6):6.1f} Hz   "
              f"최대속력 {spd.max():.2f} m/s   ({types[name]})")

    # ---------- 각 소스의 변위 ----------
    print()
    print("=== 변위 ===")
    res = {}
    for key, (name, t, xy, yaw, spd) in src.items():
        r = measure(name, t, xy, spd, args.settle, args.speed)
        if r is None:
            continue
        res[key] = r
        print(f"  [{name}]")
        print(f"    이동 구간   {r['t0']-t[0]:.1f} ~ {r['t1']-t[0]:.1f} s "
              f"({r['dur']:.1f} s)")
        print(f"    앵커 샘플   시작 {r['n0']}개, 끝 {r['n1']}개")
        print(f"    직선 chord  {r['chord']:.3f} m")
        print(f"    경로장 path {r['path']:.3f} m   "
              f"(chord 대비 {r['path']/max(r['chord'],1e-9):.3f}배)")

    if not res:
        sys.exit(1)

    # ---------- 두 소스가 같은가 ----------
    if "sport" in src and "odom" in src:
        print()
        print("=== /utlidar/robot_odom 과 /lf/sportmodestate 의 관계 ===")
        _, ts, xys, yaws, spds = src["sport"]
        _, to, xyo, yawo, spdo = src["odom"]

        # 이동 구간에서 sport 시각에 odom 을 보간
        t0 = max(res["sport"]["t0"], res["odom"]["t0"])
        t1 = min(res["sport"]["t1"], res["odom"]["t1"])
        m = (ts >= t0) & (ts <= t1)
        if m.sum() >= 20:
            ox = np.interp(ts[m], to, xyo[:, 0])
            oy = np.interp(ts[m], to, xyo[:, 1])
            O = np.column_stack([ox, oy])
            S = xys[m]

            raw = np.linalg.norm(O - S, axis=1)
            yaw_r, _, r_rigid = fit_rigid(O, S, False)
            _, s_fit, r_scaled = fit_rigid(O, S, True)

            print(f"  같은 프레임에서 직접 차이   평균 {raw.mean():.4f} m, "
                  f"최대 {raw.max():.4f} m")
            print(f"  yaw만 정렬 후 잔차 RMS      "
                  f"{np.sqrt((r_rigid**2).mean()):.4f} m  "
                  f"(상대 yaw {yaw_r:+.2f} deg)")
            print(f"  yaw+스케일 정렬 후 잔차 RMS "
                  f"{np.sqrt((r_scaled**2).mean()):.4f} m")
            print(f"  odom -> sport 스케일        {s_fit:.4f}")
            print()
            if raw.max() < 0.02:
                print("  -> 두 토픽은 사실상 동일한 값입니다.")
                print("     스케일 보정 노드 하나로 A 쪽까지 같이 해결됩니다.")
            elif abs(s_fit - 1.0) < 0.01:
                print("  -> 스케일은 같고 위치만 조금 다릅니다 "
                      "(프레임 원점 또는 지연 차이).")
                print("     k 는 한 값을 공유해도 됩니다.")
            else:
                print(f"  -> 두 소스의 스케일이 {s_fit:.4f} 배 다릅니다.")
                print("     k 를 소스별로 따로 재야 합니다.")

            ratio = res["odom"]["chord"] / max(res["sport"]["chord"], 1e-9)
            print(f"  chord 비 (odom/sport)       {ratio:.4f}")
        else:
            print(f"  겹치는 이동 구간 샘플 {m.sum()}개 — 비교 생략")

    # ---------- 실측 대비 k ----------
    print()
    print("=== 스케일 계수 k ===")
    if args.truth is None:
        print("  --truth 를 주지 않아 k 를 계산하지 않았습니다.")
        print("  줄자 실측값을 넣어 주세요:  --truth 5.00")
    else:
        print(f"  실측 직선거리 {args.truth:.3f} m")
        print()
        for key, r in res.items():
            name = src[key][0]
            k = args.truth / max(r["chord"], 1e-9)
            err = (r["chord"] - args.truth) / args.truth * 100.0
            print(f"  [{name}]")
            print(f"    측정 {r['chord']:.3f} m,  오차 {err:+.2f}%")
            print(f"    k = {args.truth:.3f} / {r['chord']:.3f} = {k:.4f}")
            if 1.15 <= k <= 1.27:
                print(f"    -> 기존 실외 추정 1.19~1.23 과 같은 범위입니다.")
            elif abs(k - 1.0) < 0.03:
                print(f"    -> 보정이 거의 필요 없습니다.")
            else:
                print(f"    -> 기존 범위(1.19~1.23) 밖입니다. "
                      f"보행 모드·노면을 확인해 주세요.")
        print()
        print("  주의: 1회 측정으로 확정하지 마시고 최소 3회 반복한 뒤")
        print("        중앙값을 쓰시는 편이 안전합니다.")

    # ---------- 그림 ----------
    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("\nmatplotlib 없음 — 그림 생략")
            return

        fig, ax = plt.subplots(1, 2, figsize=(13, 5.2))
        colors = {"sport": "#639922", "odom": "#378ADD"}
        for key, (name, t, xy, yaw, spd) in src.items():
            ax[0].plot(xy[:, 0], xy[:, 1], lw=1.3,
                       color=colors.get(key, "#888780"), label=name)
            if key in res:
                r = res[key]
                ax[0].scatter(*r["p0"], marker="s", s=55,
                              color=colors.get(key, "#888780"), zorder=5)
                ax[0].scatter(*r["p1"], marker="X", s=70,
                              color=colors.get(key, "#888780"), zorder=5)
            ax[1].plot(t - t[0], spd, lw=1.0,
                       color=colors.get(key, "#888780"), label=name)

        ax[0].set_xlabel("x [m]")
        ax[0].set_ylabel("y [m]")
        ax[0].set_title("trajectory (square = start anchor, X = end anchor)")
        ax[0].axis("equal")
        ax[0].grid(alpha=0.25)
        ax[0].legend(fontsize=8)

        ax[1].axhline(args.speed, ls="--", lw=0.8, color="#E24B4A",
                      label=f"move threshold {args.speed}")
        ax[1].set_xlabel("time since bag start [s]")
        ax[1].set_ylabel("speed [m/s]")
        ax[1].set_title("speed")
        ax[1].grid(alpha=0.25)
        ax[1].legend(fontsize=8)

        fig.tight_layout()
        out = "odom_scale_check.png"
        fig.savefig(out, dpi=140)
        print(f"\n저장: {out}")


if __name__ == "__main__":
    main()
