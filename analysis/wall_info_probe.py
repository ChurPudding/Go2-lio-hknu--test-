#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wall_info_probe.py — 원본 /utlidar/cloud 에서 '벽 점'만 골라
                     yaw 를 잡을 정보가 프레임마다 얼마나 있는지 정량화한다.

왜 필요한가
  L1 은 164.9° 기울어 달려 스캔의 대부분이 바닥이다. 바닥 점은 아무리 많아도
  z·roll·pitch 만 잡고 yaw·수평이동은 못 잡는다. yaw 를 잡는 건 '벽 점'뿐이다.
  게다가 벽 점이 많아도 그 법선이 다 한 방향(복도)이면 yaw 정보는 0 이다.
  그래서 '총 점 수'가 아니라 다음을 봐야 한다:
      (1) 벽 점 개수        — 벽을 얼마나 봤나
      (2) 법선 방향 다양성  — 그 벽들이 몇 갈래 방향을 담고 있나  ← 핵심

정보량 지표 (조건화와 직접 연결)
  각 벽 점의 '수평 법선 방향'을 모아 2x2 산포행렬 C 를 만든다.
      C = (1/N) Σ  n_h n_h^T          (n_h = 법선의 xy 성분, 정규화)
  C 의 두 고유값 λ1 ≥ λ2 로 방향 다양성을 잰다.
      · 한 방향뿐(복도)  → λ2 ≈ 0        → yaw 관측 취약 (퇴화)
      · 두 방향 이상     → λ2 가 큼        → yaw 잘 잡힘
  지표 aniso = λ2 / λ1  (0=완전퇴화 ~ 1=등방).  aniso 가 작으면 퇴화.

바닥/벽 분리
  raw 점을 R_LB 로 몸통(월드근사) 프레임으로 돌린 뒤 높이 z_b 로 나눈다.
      p_body = R_LB^T · p_lidar      (R_LB = 몸통→라이다 이므로 역이 라이다→몸통)
  z_b 가 로봇 기준 --wall-z 보다 위면 벽 후보. (기본 -0.3 m: 로봇 몸통보다 위)

법선 추정
  프레임 내에서 각 벽 점의 k 최근접으로 국소 평면 법선을 구한다(작은 k, 빠르게).
  open3d 없이 numpy 로만.

사용
  python3 wall_info_probe.py ~/data/bags/lio_test_bag_loop_run1 \
        --traj aft_off25.csv --traj-rate 0.5
옵션
  --wall-z -0.3     벽 판정 높이(로봇 기준, m). 올리면 더 확실한 벽만
  --stride 3        프레임 N개마다 1개만 처리(속도). 기본 3
  --k 8             법선 추정 최근접 개수
"""

import argparse
import glob
import os
import sys
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# go2_calib 의 R_LB (몸통->라이다). 노트북 값 하드코딩(컨테이너에 go2_calib 없음).
R_LB = np.array([
    [ 0.523029, -0.838576,  0.15242 ],
    [-0.810712, -0.544668, -0.214668],
    [ 0.263034, -0.011292, -0.964721],
])
R_BL = R_LB.T          # 라이다 -> 몸통


def read_cloud_xyz(m):
    """PointCloud2 -> (N,3) float32 xyz. point_step 32, x/y/z offset 0/4/8."""
    buf = np.frombuffer(bytes(m.data), dtype=np.uint8)
    n = m.width * m.height
    buf = buf.reshape(n, m.point_step)
    xyz = np.zeros((n, 3), np.float32)
    xyz[:, 0] = buf[:, 0:4].copy().view(np.float32).ravel()
    xyz[:, 1] = buf[:, 4:8].copy().view(np.float32).ravel()
    xyz[:, 2] = buf[:, 8:12].copy().view(np.float32).ravel()
    # 유효점만 (NaN/0,0,0 제거)
    ok = np.isfinite(xyz).all(1) & (np.abs(xyz).sum(1) > 1e-3)
    return xyz[ok]


def wall_normals_aniso(pw, k=8, max_pts=1200):
    """벽 점 pw(N,3, 몸통프레임)의 수평 법선 산포 고유값비 aniso 반환."""
    N = len(pw)
    if N < 20:
        return N, float('nan'), float('nan')
    # 속도: 점이 많으면 샘플링
    if N > max_pts:
        idx = np.random.default_rng(0).choice(N, max_pts, replace=False)
        pw = pw[idx]; N = max_pts
    # 각 점 법선: k 최근접의 공분산 최소고유벡터 (numpy brute force)
    # 거리행렬은 N^2 이라 max_pts 로 제한. 수평성분만 필요.
    nrm = np.zeros((N, 3))
    # KD 없이: 랜덤 부분집합으로 근사하면 부정확 → 전수 최근접(제한된 N에서만)
    D = np.linalg.norm(pw[:, None, :] - pw[None, :, :], axis=2)
    for i in range(N):
        nn = np.argsort(D[i])[:k + 1]
        Q = pw[nn] - pw[nn].mean(0)
        w, v = np.linalg.eigh(Q.T @ Q)
        nrm[i] = v[:, 0]                 # 최소고유값 방향 = 법선
    nh = nrm[:, :2]                      # 수평성분
    ln = np.linalg.norm(nh, axis=1)
    nh = nh[ln > 1e-6] / ln[ln > 1e-6, None]
    if len(nh) < 10:
        return N, float('nan'), float('nan')
    C = (nh.T @ nh) / len(nh)            # 2x2 산포
    ev = np.sort(np.linalg.eigvalsh(C))  # [λ2, λ1]
    lam2, lam1 = float(ev[0]), float(ev[1])
    aniso = lam2 / lam1 if lam1 > 1e-9 else 0.0
    return N, aniso, lam2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/utlidar/cloud")
    ap.add_argument("--wall-z", type=float, default=-0.3,
                    help="이 높이(로봇기준 m)보다 위면 벽 후보")
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--k", type=int, default=8)
    ap.add_argument("--traj", default=None)
    ap.add_argument("--traj-rate", type=float, default=0.5)
    args = ap.parse_args()

    d = args.bag
    if os.path.isdir(d):
        d = glob.glob(os.path.join(d, "*.db3"))[0]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=d, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    M = get_message(types[args.topic])

    traj = None
    if args.traj:
        a = np.genfromtxt(args.traj, delimiter=",", skip_header=1)
        traj = (a[:, 0] * args.traj_rate, a[:, 1], a[:, 2])

    print("=" * 70)
    print(f"벽 점 정보 분석  {args.topic}  (wall_z>{args.wall_z}, stride={args.stride})")
    print("aniso = 법선방향 다양성 (0=한방향/퇴화, 1=등방/양호)")
    print("=" * 70)

    rows = []          # (t, n_total, n_wall, wall_frac, aniso)
    t0 = None
    fi = -1
    while r.has_next():
        tp, data, t_ns = r.read_next()
        if tp != args.topic:
            continue
        fi += 1
        if fi % args.stride != 0:
            continue
        m = deserialize_message(data, M)
        t = t_ns * 1e-9
        if t0 is None:
            t0 = t
        xyz = read_cloud_xyz(m)
        if len(xyz) < 20:
            continue
        pb = (R_BL @ xyz.T).T           # 라이다 -> 몸통
        wall = pb[pb[:, 2] > args.wall_z]
        nw, aniso, lam2 = wall_normals_aniso(wall, k=args.k)
        rows.append((t - t0, len(xyz), len(wall),
                     len(wall) / len(xyz), aniso))

    A = np.array(rows)
    if len(A) == 0:
        sys.exit("처리된 프레임 0개")

    tt, ntot, nwall, wfrac, ani = A.T
    print(f"프레임 {len(A)}개 처리")
    print(f"총점 중앙값 {np.median(ntot):.0f} | 벽점 중앙값 {np.median(nwall):.0f} "
          f"({100*np.median(wfrac):.1f}%)")
    print(f"aniso 중앙값 {np.nanmedian(ani):.3f}")
    print()

    # 퇴화 프레임: 벽점 적거나 aniso 낮음
    ani_thr = np.nanmedian(ani) * 0.4
    print(f"── 퇴화 의심 구간 (aniso < {ani_thr:.3f}, 즉 법선이 한 방향에 쏠림) ──")
    bad = ani < ani_thr
    i = 0
    n = len(bad)
    any_seg = False
    while i < n:
        if bad[i] and not np.isnan(ani[i]):
            j = i
            while j < n and bad[j]:
                j += 1
            dur = tt[j - 1] - tt[i]
            if dur >= 1.0:
                any_seg = True
                loc = ""
                if traj is not None:
                    tc = (tt[i] + tt[j - 1]) / 2
                    kk = int(np.argmin(np.abs(traj[0] - tc)))
                    loc = f"  위치 ({traj[1][kk]:+.1f},{traj[2][kk]:+.1f})"
                print(f"  {tt[i]:6.1f}~{tt[j-1]:6.1f}s ({dur:4.1f}s)  "
                      f"aniso~{np.nanmean(ani[i:j]):.3f}  "
                      f"벽점~{int(np.nanmean(nwall[i:j]))}{loc}")
            i = j
        else:
            i += 1
    if not any_seg:
        print("  없음 — 벽 방향이 대체로 다양. 순수 기하 퇴화는 국소적.")

    out = "wall_info.csv"
    np.savetxt(out, A, delimiter=",",
               header="t_sec_bag,n_total,n_wall,wall_frac,aniso",
               comments="", fmt="%.4f,%d,%d,%.4f,%.4f")
    print(f"\n저장: {out}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        ax[0].plot(tt, nwall, lw=0.8, label="wall points")
        ax[0].plot(tt, ntot, lw=0.5, alpha=0.4, label="total points")
        ax[0].set_ylabel("points"); ax[0].legend(fontsize=8)
        ax[0].set_title("wall point count")
        ax[1].plot(tt, ani, lw=0.8, color="purple")
        ax[1].axhline(ani_thr, color="r", ls=":", lw=0.8,
                      label=f"degenerate thr {ani_thr:.3f}")
        ax[1].set_ylabel("aniso (normal diversity)")
        ax[1].set_xlabel("t_sec (bag)")
        ax[1].set_title("yaw-info: normal direction diversity  (low = degenerate)")
        ax[1].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig("wall_info.png", dpi=130)
        print("그림: wall_info.png")
    except Exception as e:
        print(f"(그림 생략: {e})")


if __name__ == "__main__":
    main()
