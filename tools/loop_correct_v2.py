#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loop_correct_v2.py — 증분 재적분 방식 루프 클로저

  왜 v1 이 틀렸나
  --------------
  v1 은 T_a = exp(a·log(T_lc)) 로 세계 좌표를 통째로 변환했다.
  SE(2) 에서 회전 성분이 있으면 이것은 '나선 운동'이 되어, 어딘가 고정된
  회전 중심이 생기고 거기서 먼 점일수록 크게 밀린다.
  20.9° 회전이면 중심에서 50 m 떨어진 점은 18 m 나 움직인다.
  실제 드리프트는 그렇게 생기지 않는다 — 걸어가면서 조금씩 쌓인다.

  v2 가 하는 일
  ------------
    1. 연속 자세를 상대 변환으로 분해      dX_k = X_{k-1}^-1 · X_k
    2. 각 걸음의 회전에 delta 를 더한다     (이동 거리에 비례 배분)
    3. 다시 적분해서 궤적 재구성            → 국소 모양 보존
    4. 남은 위치 오차만 비례 분배           → 최대 이동량 = 오차 크기

  각 점군 프레임에는 그 시점의 (옛 자세 → 새 자세) 변환을 적용한다.
      T_k = X'_k · X_k^-1

  사용 예:
    # 출발점에 정확히, 같은 방향으로 복귀 (0805)
    python3 loop_correct_v2.py ~/data/bags/indoor/floor_0805_1720

    # 반대 방향으로 끝남 (0807)
    python3 loop_correct_v2.py ~/data/bags/indoor/floor_0807_1542 --end-dyaw 180

    # 궤적만 확인
    python3 loop_correct_v2.py ~/data/bags/indoor/floor_0805_1720 --plot-only
"""

import argparse
import math
import os

import numpy as np
import yaml


def detect_storage(bag_dir):
    meta = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.exists(meta):
        with open(meta) as f:
            m = yaml.safe_load(f)
        try:
            return m['rosbag2_bagfile_information']['storage_identifier']
        except Exception:
            pass
    for f in os.listdir(bag_dir):
        if f.endswith('.mcap'):
            return 'mcap'
    return 'sqlite3'


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rot(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def open_reader(bag):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=detect_storage(bag)),
           ConverterOptions('', ''))
    return r


def deform(P, TH, s, target_p, target_th):
    """증분 재적분으로 궤적을 변형한다.

    P       : (N,2) 위치
    TH      : (N,)  yaw
    s       : (N,)  누적 이동 거리
    target_* : 마지막 자세가 가져야 할 값

    반환: P2 (N,2), TH2 (N,)
    """
    n = len(P)
    stot = s[-1] if s[-1] > 0 else 1.0

    # 1) 상대 변환으로 분해
    dp = np.zeros((n, 2))
    dth = np.zeros(n)
    for k in range(1, n):
        dp[k] = rot(TH[k - 1]).T @ (P[k] - P[k - 1])
        dth[k] = wrap(TH[k] - TH[k - 1])

    # 2) 회전 오차를 이동 거리에 비례해 각 걸음에 배분
    dyaw_tot = wrap(target_th - TH[-1])
    ds = np.diff(s, prepend=s[0])
    dth2 = dth + dyaw_tot * (ds / stot)

    # 3) 재적분 (국소 모양은 그대로, 방향만 서서히 틀어짐)
    TH2 = np.zeros(n)
    P2 = np.zeros((n, 2))
    TH2[0], P2[0] = TH[0], P[0]
    for k in range(1, n):
        TH2[k] = TH2[k - 1] + dth2[k]
        P2[k] = P2[k - 1] + rot(TH2[k - 1]) @ dp[k]

    # 4) 남은 위치 오차만 비례 분배 (회전 없음 → 최대 이동량이 오차로 제한)
    e = target_p - P2[-1]
    P2 = P2 + (s / stot)[:, None] * e
    return P2, TH2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', nargs='?',
                    default=os.path.expanduser('~/data/bags/indoor/floor_0805_1720'))
    ap.add_argument('--voxel', type=float, default=0.05)
    ap.add_argument('--end-dx', type=float, default=0.0)
    ap.add_argument('--end-dy', type=float, default=0.0)
    ap.add_argument('--end-dyaw', type=float, default=0.0)
    ap.add_argument('--out', default=os.path.expanduser(
        '~/fastlio_ws/results/odommap_v2'))
    ap.add_argument('--plot-only', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    CLOUD = '/utlidar/cloud_deskewed'
    ODOM = '/utlidar/robot_odom'

    r = open_reader(a.bag)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    odom_cls = get_message(types[ODOM])
    cloud_cls = get_message(types[CLOUD])

    # ---------- 오도메트리 ----------
    print('[1] 오도메트리 읽는 중...')
    T, P, TH = [], [], []
    while r.has_next():
        n, d, _ = r.read_next()
        if n != ODOM:
            continue
        m = deserialize_message(d, odom_cls)
        p = m.pose.pose.position
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        P.append((p.x, p.y))
        TH.append(yaw_of(m.pose.pose.orientation))
    T = np.array(T)
    P = np.array(P)
    TH = np.array(TH)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    print(f'    {len(T):,} 개,  총 이동 {s[-1]:.1f} m')

    # ---------- 목표 자세 ----------
    target_th = TH[0] + math.radians(a.end_dyaw)
    target_p = P[0] + rot(TH[0]) @ np.array([a.end_dx, a.end_dy])

    meas_d = float(np.linalg.norm(P[-1] - P[0]))
    meas_yaw = math.degrees(wrap(TH[-1] - TH[0]))
    err_p = float(np.linalg.norm(target_p - P[-1]))
    err_yaw = math.degrees(wrap(target_th - TH[-1]))

    print('\n[2] 보정할 오차\n')
    print('| 항목 | 측정 | 실측(입력) | 오차 |')
    print('|---|---|---|---|')
    print(f'| 끝 위치 (시작 기준) | {meas_d:.2f} m | '
          f'{math.hypot(a.end_dx, a.end_dy):.2f} m | **{err_p:.2f} m** |')
    print(f'| 끝 방향 (시작 대비) | {meas_yaw:+.1f}° | {a.end_dyaw:+.1f}° | '
          f'**{err_yaw:+.1f}°** |')

    # ---------- 변형 ----------
    P2, TH2 = deform(P, TH, s, target_p, target_th)

    span_raw = P.max(0) - P.min(0)
    span_new = P2.max(0) - P2.min(0)
    print(f'\n[3] 궤적 크기 (변형 전 → 후)')
    print(f'    x  {span_raw[0]:.1f} → {span_new[0]:.1f} m')
    print(f'    y  {span_raw[1]:.1f} → {span_new[1]:.1f} m')
    ratio = max(span_new / np.maximum(span_raw, 1e-6))
    if ratio > 1.3:
        print(f'    ⚠ 궤적이 {ratio:.2f} 배로 커졌습니다. 입력값을 확인하세요.')
    else:
        print(f'    최대 변화 {ratio:.2f} 배 — 모양이 보존됐습니다.')

    # ---------- 그림 ----------
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(15, 7))
        sc = ax[0].scatter(P[:, 0], P[:, 1], c=T - T[0], s=1, cmap='viridis')
        ax[0].plot(*P[0], 'go', ms=12, label='start')
        ax[0].plot(*P[-1], 'rx', ms=14, mew=3, label='end')
        ax[0].set_title(f'raw   start-end {meas_d:.2f} m,  yaw {meas_yaw:+.1f} deg')
        ax[0].legend()
        plt.colorbar(sc, ax=ax[0], label='elapsed [s]')

        ax[1].plot(P[:, 0], P[:, 1], lw=1, c='tab:red', alpha=.55, label='raw')
        ax[1].plot(P2[:, 0], P2[:, 1], lw=1, c='tab:blue', label='corrected')
        ax[1].plot(*P2[0], 'go', ms=10)
        ax[1].plot(*P2[-1], 'bx', ms=12, mew=3)
        ax[1].set_title('raw vs corrected (incremental)')
        ax[1].legend()
        for x in ax:
            x.set_aspect('equal'); x.grid(alpha=.3)
            x.set_xlabel('x [m]'); x.set_ylabel('y [m]')
        plt.tight_layout()
        p = os.path.join(a.out, 'trajectory.png')
        plt.savefig(p, dpi=115); plt.close()
        print(f'\n[4] 궤적 그림: {p}')
    except Exception as e:
        print(f'\n[4] (그림 생략: {e})')

    if a.plot_only:
        print('\n--plot-only 지정. 지도 생성은 건너뜁니다.')
        return

    # ---------- 지도 재구성 ----------
    import open3d as o3d
    print('\n[5] 보정 적용하며 지도 재구성...')
    r = open_reader(a.bag)
    total = 0
    while r.has_next():
        n, _, _ = r.read_next()
        if n == CLOUD:
            total += 1
    half = total // 2

    r = open_reader(a.bag)
    A = o3d.geometry.PointCloud()
    B = o3d.geometry.PointCloud()
    bufA, bufB = [], []
    k = 0

    def flush(buf, pc):
        if not buf:
            return pc
        q = o3d.geometry.PointCloud()
        q.points = o3d.utility.Vector3dVector(np.vstack(buf))
        pc += q
        return pc.voxel_down_sample(a.voxel)

    while r.has_next():
        n, d, _ = r.read_next()
        if n != CLOUD:
            continue
        m = deserialize_message(d, cloud_cls)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        i = int(np.searchsorted(T, stamp))
        i = max(0, min(i, len(T) - 1))

        # 이 시점의 (옛 자세 → 새 자세) 변환
        R = rot(TH2[i] - TH[i])
        t = P2[i] - R @ P[i]

        arr = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'),
                                             skip_nans=True)
        arr = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
        arr = arr[np.isfinite(arr).all(axis=1)]
        if len(arr):
            xy = (R @ arr[:, :2].T).T + t
            arr = np.column_stack([xy, arr[:, 2]])   # z 는 보정하지 않는다
            (bufA if k < half else bufB).append(arr)
        k += 1
        if k % 400 == 0:
            if k <= half:
                A = flush(bufA, A); bufA = []
            else:
                B = flush(bufB, B); bufB = []
            print(f'    {k}/{total}')

    A = flush(bufA, A)
    B = flush(bufB, B)

    print('\n[6] 전반/후반 겹침 측정...')
    res = o3d.pipelines.registration.registration_icp(
        B.voxel_down_sample(0.2), A.voxel_down_sample(0.2), 3.0, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    sh = float(np.linalg.norm(res.transformation[:3, 3]))
    yw = math.degrees(math.atan2(res.transformation[1, 0],
                                 res.transformation[0, 0]))
    print(f'    평행이동 {sh:.2f} m, 회전 {yw:+.2f}°, RMSE {res.inlier_rmse:.3f} m')

    M = (A + B).voxel_down_sample(a.voxel)
    pts = np.asarray(M.points)
    print(f'\n최종 지도 {len(pts):,} 점\n')
    print('| 항목 | 값 |')
    print('|---|---|')
    for i, ax_ in enumerate('xyz'):
        v = pts[:, i]
        print(f'| {ax_} p1~p99 | {np.percentile(v,99)-np.percentile(v,1):.2f} m |')
    hist, edges = np.histogram(pts[:, 2], bins=100)
    kk = int(np.argmax(hist))
    print(f'| 지면 추정 z | {(edges[kk]+edges[kk+1])/2:.3f} m |')

    out = os.path.join(a.out, 'scans.pcd')
    o3d.io.write_point_cloud(out, M)
    print(f'\n저장: {out}')
    print('\n다음:')
    print(f'  python3 ~/fastlio_ws/tools/pcd_to_grid.py \\')
    print(f'    {out} {a.out}/grid 0.10')


if __name__ == '__main__':
    main()
