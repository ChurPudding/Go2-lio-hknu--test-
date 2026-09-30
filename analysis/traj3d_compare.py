#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traj3d_compare.py

출력 bag에서 Point-LIO(/aft_mapped_to_init)와 다리 오도메트리(/utlidar/robot_odom)의
3D 궤적을 뽑아, 회전 가능한 3D 그래프로 겹쳐 그린다.
정지 구간(다리 속도 < 임계)은 굵은 점으로 강조 → "정지 중 궤적이 어떻게 뭉치거나
소용돌이치는가"를 공간에서 직접 확인.

사용:
  python3 traj3d_compare.py ~/data/bags/plout_ekf1_zupton_0147
  python3 traj3d_compare.py <bag> --still-speed 0.05 --out traj3d.png
  python3 traj3d_compare.py <bag> --interactive     # 창을 띄워 마우스로 회전
  python3 traj3d_compare.py <bag> --align           # 시작 위치/방향을 맞춰 겹쳐보기
"""

import sys
import math
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt          # 3D 축 등록을 위해 먼저 import
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401  (등록 목적)

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

PL_TOPIC = "/aft_mapped_to_init"     # Point-LIO 출력
LEG_TOPIC = "/utlidar/robot_odom"    # 다리 오도메트리


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def stamp_sec(msg):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


def read_bag(path):
    """PL/다리 궤적을 (t, x, y, z, yaw) 배열로 읽는다."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in (PL_TOPIC, LEG_TOPIC):
        if need not in types:
            sys.exit(f"[에러] bag에 {need} 가 없습니다. 담긴 토픽: {list(types)}")

    MsgPL = get_message(types[PL_TOPIC])
    MsgLeg = get_message(types[LEG_TOPIC])
    pl, leg = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == PL_TOPIC:
            m = deserialize_message(data, MsgPL)
            p = m.pose.pose.position
            pl.append((stamp_sec(m), p.x, p.y, p.z,
                       yaw_from_quat(m.pose.pose.orientation)))
        elif topic == LEG_TOPIC:
            m = deserialize_message(data, MsgLeg)
            p = m.pose.pose.position
            leg.append((stamp_sec(m), p.x, p.y, p.z,
                        yaw_from_quat(m.pose.pose.orientation)))
    if not pl:
        sys.exit(f"[에러] {PL_TOPIC} 메시지가 0개입니다.")
    if not leg:
        sys.exit(f"[에러] {LEG_TOPIC} 메시지가 0개입니다.")
    return np.array(pl), np.array(leg)


def still_mask(t, x, y, v_th):
    """위치 미분으로 정지 마스크(True=정지)를 만든다."""
    dt = np.clip(np.gradient(t), 1e-3, None)
    speed = np.hypot(np.gradient(x), np.gradient(y)) / dt
    return speed < v_th


def rot2d(x, y, ang):
    c, s = math.cos(ang), math.sin(ang)
    return c * x - s * y, s * x + c * y


def align_to(ref_xy0, ref_yaw0, x, y, yaw0):
    """다리 궤적을 PL 시작점/시작방향에 맞춰 회전·평행이동 (겹쳐보기용)."""
    dyaw = ref_yaw0 - yaw0
    xr, yr = rot2d(x - x[0], y - y[0], dyaw)
    return xr + ref_xy0[0], yr + ref_xy0[1]


def set_equal_3d(ax, xs, ys, zs):
    """3D 축 비율을 실제 거리와 맞춘다 (안 맞추면 궤적이 왜곡돼 보임)."""
    xr = xs.max() - xs.min()
    yr = ys.max() - ys.min()
    zr = zs.max() - zs.min()
    r = max(xr, yr, zr, 1e-3) / 2.0
    cx, cy, cz = xs.mean(), ys.mean(), zs.mean()
    ax.set_xlim(cx - r, cx + r)
    ax.set_ylim(cy - r, cy + r)
    ax.set_zlim(cz - r, cz + r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="출력 bag 경로")
    ap.add_argument("--still-speed", type=float, default=0.05,
                    help="정지 판정 속도 임계값 [m/s] (기본 0.05)")
    ap.add_argument("--align", action="store_true",
                    help="다리 궤적을 PL 시작점/방향에 맞춰 겹쳐 그림")
    ap.add_argument("--interactive", action="store_true",
                    help="PNG 저장 대신 회전 가능한 창을 띄움")
    ap.add_argument("--out", default="traj3d.png", help="그림 저장 경로")
    args = ap.parse_args()

    if not args.interactive:
        matplotlib.use("Agg")

    pl, leg = read_bag(args.bag)
    pl_t, pl_x, pl_y, pl_z = pl[:, 0], pl[:, 1], pl[:, 2], pl[:, 3]
    leg_t, leg_x, leg_y, leg_z, leg_yaw = (leg[:, 0], leg[:, 1], leg[:, 2],
                                           leg[:, 3], leg[:, 4])

    pl_still = still_mask(pl_t, pl_x, pl_y, args.still_speed)
    leg_still = still_mask(leg_t, leg_x, leg_y, args.still_speed)

    # 겹쳐보기: 다리 궤적을 PL 시작점/방향에 정렬
    if args.align:
        leg_x, leg_y = align_to((pl_x[0], pl_y[0]), pl[0, 4],
                                leg_x, leg_y, leg_yaw[0])
        leg_z = leg_z - leg_z[0] + pl_z[0]

    # ── 콘솔 요약: 정지 중 두 궤적이 얼마나 "제자리"였는가
    def wander(x, y, mask):
        if mask.sum() < 2:
            return 0.0
        return float(np.hypot(x[mask] - x[mask].mean(),
                              y[mask] - y[mask].mean()).max())
    print("=" * 56)
    print(f"bag              : {args.bag}")
    print(f"PL / 다리 샘플    : {len(pl_t)} / {len(leg_t)}")
    print(f"정지 판정 임계값  : {args.still_speed} m/s")
    print(f"정지 중 최대 이탈 (제자리서 얼마나 벗어났나):")
    print(f"   Point-LIO : {wander(pl_x, pl_y, pl_still):.3f} m")
    print(f"   다리       : {wander(leg_x, leg_y, leg_still):.3f} m")
    print("   (정지인데 이 값이 크면 그 구간에서 궤적이 흘렀다는 뜻)")
    print("=" * 56)

    # ── 3D 플롯
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")

    # 이동 궤적 (선)
    ax.plot(pl_x, pl_y, pl_z, color="#D85A30", lw=1.3, label="Point-LIO")
    ax.plot(leg_x, leg_y, leg_z, color="#1D9E75", lw=1.3, label="Leg odom")

    # 정지 구간 (굵은 점) — 여기가 붕괴가 드러나는 곳
    ax.scatter(pl_x[pl_still], pl_y[pl_still], pl_z[pl_still],
               color="#A32D2D", s=8, depthshade=False, label="PL standstill")
    ax.scatter(leg_x[leg_still], leg_y[leg_still], leg_z[leg_still],
               color="#0F6E56", s=8, depthshade=False, label="leg standstill")

    # 시작점 표시
    ax.scatter([pl_x[0]], [pl_y[0]], [pl_z[0]], color="k", s=40, marker="^")

    allx = np.concatenate([pl_x, leg_x])
    ally = np.concatenate([pl_y, leg_y])
    allz = np.concatenate([pl_z, leg_z])
    set_equal_3d(ax, allx, ally, allz)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("3D trajectory: Point-LIO vs leg odom  (dots = standstill)")
    ax.legend(loc="upper left", fontsize=9)
    ax.view_init(elev=25, azim=-60)   # 초기 시점

    if args.interactive:
        plt.show()
    else:
        fig.tight_layout()
        fig.savefig(args.out, dpi=130)
        print(f"그림 저장: {args.out}")


if __name__ == "__main__":
    main()
