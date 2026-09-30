#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PL_loop_closure.py — Point-LIO 궤적 루프 클로저 (두 bag 정렬)

  loop_correct_v2.py 와의 차이
  ---------------------------
  v2 는 한 bag 에서 궤적(/utlidar/robot_odom, 다리 오도)과
  점군(/utlidar/cloud_deskewed)을 같이 읽었다.
  이 스크립트는 Point-LIO 궤적을 루프 클로저 하기 위해 두 가지를 바꾼다.

    1. 두 bag 분리
         --traj-bag   Point-LIO 궤적 (/aft_mapped_to_init) — 재생·녹화한 bag
         --cloud-bag  원본 점군 (/utlidar/cloud)           — 원본 bag
    2. header.stamp 로 정렬
         v2 는 bag 기록시각(t_ns)으로 짝지었다. 두 bag 은 기록시각이
         전혀 다르므로(재생 시각 vs 원본 시각) 쓸 수 없다. 대신
         Point-LIO 가 입력 점군의 header.stamp 를 출력에 그대로 넣으므로,
         두 bag 의 header.stamp 는 같은 시계다. 이걸로 짝짓는다.
             i* = argmin_i | t_traj[i] - t_cloud[j] |
    3. deskewed 대신 원본 점군
         /utlidar/cloud_deskewed 는 88.6% 중복점이라 원본 /utlidar/cloud 를 쓴다.

  보정 로직(deform)은 v2 와 동일한 '증분 재적분'이다.
    1. 연속 자세를 상대 변환으로 분해   dX_k = X_{k-1}^-1 · X_k
    2. 회전 오차를 이동거리에 비례해 각 걸음에 배분
    3. 다시 적분해 궤적 재구성            → 국소 모양 보존
    4. 남은 위치 오차만 비례 분배          → 최대 이동량 = 오차 크기

  폐루프(2바퀴 복귀)면 목표는 '시작점 = 끝점' 이므로 end-dx=end-dy=0 (기본).

  사용 예:
    # 궤적만 보정 (점군 지도 생략, 빠름)
    python3 PL_loop_closure.py \\
        --traj-bag  ~/data/bags/run_PL_run2_b \\
        --plot-only

    # 궤적 보정 + 점군 지도 재구성
    python3 PL_loop_closure.py \\
        --traj-bag  ~/data/bags/run_PL_run2_b \\
        --cloud-bag ~/data/bags/lio_test_bag_loop_run2

    # 폐루프가 아니라 끝점 오차를 직접 지정할 때
    python3 PL_loop_closure.py --traj-bag <bag> --end-dx 13.29 --end-dy 38.84
"""

import argparse
import math
import os

import numpy as np
import yaml


TRAJ_TOPIC = '/aft_mapped_to_init'
CLOUD_TOPIC = '/cloud_registered'   # Point-LIO 가 world 좌표로 내보내는 정합점군


# ─────────────────────────────────────────────────────────────
# bag 유틸
# ─────────────────────────────────────────────────────────────
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


def open_reader(bag):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    sid = detect_storage(bag) if os.path.isdir(bag) else 'sqlite3'
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=sid),
           rosbag2_py.ConverterOptions('', ''))
    return r


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rot(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


# ─────────────────────────────────────────────────────────────
# 궤적 변형 (v2 와 동일)
# ─────────────────────────────────────────────────────────────
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
    # 4) 남은 위치 오차만 비례 분배
    e = target_p - P2[-1]
    P2 = P2 + (s / stot)[:, None] * e
    return P2, TH2


# ─────────────────────────────────────────────────────────────
# 궤적 읽기 (header.stamp 포함)
# ─────────────────────────────────────────────────────────────
def read_traj(bag, topic):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = open_reader(bag)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f'[에러] {bag} 에 {topic} 없음. 있는 토픽: {list(types)}')
    cls = get_message(types[topic])
    T, P, TH = [], [], []
    while r.has_next():
        n, d, _ = r.read_next()
        if n != topic:
            continue
        m = deserialize_message(d, cls)
        p = m.pose.pose.position
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        P.append((p.x, p.y))
        TH.append(yaw_of(m.pose.pose.orientation))
    return np.array(T), np.array(P), np.array(TH)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--traj-bag', required=True,
                    help='Point-LIO 궤적 bag (/aft_mapped_to_init)')
    ap.add_argument('--cloud-bag', default=None,
                    help='원본 점군 bag (/utlidar/cloud). 생략하면 궤적만 보정')
    ap.add_argument('--traj-topic', default=TRAJ_TOPIC)
    ap.add_argument('--cloud-topic', default=CLOUD_TOPIC)
    ap.add_argument('--voxel', type=float, default=0.05)
    ap.add_argument('--end-dx', type=float, default=0.0,
                    help='끝점을 시작점으로 되돌리는 x 보정량. 폐루프면 0')
    ap.add_argument('--end-dy', type=float, default=0.0)
    ap.add_argument('--end-dyaw', type=float, default=0.0)
    ap.add_argument('--out', default=os.path.expanduser(
        '~/fastlio_ws/results/pl_loopclose'))
    ap.add_argument('--plot-only', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    # ---------- 궤적 ----------
    print('[1] Point-LIO 궤적 읽는 중...')
    T, P, TH = read_traj(a.traj_bag, a.traj_topic)
    if len(T) < 10:
        raise SystemExit(f'[에러] 궤적 메시지가 {len(T)}개뿐입니다')
    s = np.concatenate([[0.0],
                        np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    print(f'    {len(T):,} 개,  총 이동 {s[-1]:.1f} m')
    print(f'    stamp {T[0]:.3f} ~ {T[-1]:.3f}  ({T[-1]-T[0]:.1f} s)')

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
    print(f'| 끝 방향 (시작 대비) | {meas_yaw:+.1f}^\\circ | {a.end_dyaw:+.1f}^\\circ | '
          f'**{err_yaw:+.1f}^\\circ** |')

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

    # 보정 후 남은 폐루프 오차 (0에 가까워야 함)
    loop_after = float(np.linalg.norm(P2[-1] - P2[0]))
    print(f'    보정 후 시작-끝 거리: {loop_after:.3f} m (보정 전 {meas_d:.2f} m)')

    # 보정된 궤적 CSV 저장
    csv = os.path.join(a.out, 'trajectory_corrected.csv')
    with open(csv, 'w') as f:
        f.write('t_sec,x_raw,y_raw,yaw_raw,x_cor,y_cor,yaw_cor\n')
        for i in range(len(T)):
            f.write('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n' % (
                T[i], P[i, 0], P[i, 1], TH[i], P2[i, 0], P2[i, 1], TH2[i]))
    print(f'    보정 궤적 CSV: {csv}')

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
        ax[1].set_title(f'raw vs corrected (loop {meas_d:.1f} -> {loop_after:.2f} m)')
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

    if a.plot_only or a.cloud_bag is None:
        if a.cloud_bag is None and not a.plot_only:
            print('\n--cloud-bag 미지정. 지도 재구성은 건너뜁니다.')
        else:
            print('\n--plot-only 지정. 지도 생성은 건너뜁니다.')
        return

    # ---------- 지도 재구성 (두 bag 정렬) ----------
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2
    import open3d as o3d

    print('\n[5] 점군 bag 정렬하며 지도 재구성...')
    r = open_reader(a.cloud_bag)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if a.cloud_topic not in types:
        raise SystemExit(f'[에러] {a.cloud_bag} 에 {a.cloud_topic} 없음. '
                         f'있는 토픽: {list(types)}')
    cloud_cls = get_message(types[a.cloud_topic])

    # 전반/후반 나누기용: 궤적 범위 안에 드는 점군 개수 먼저 세기
    r = open_reader(a.cloud_bag)
    stamps = []
    while r.has_next():
        n, d, _ = r.read_next()
        if n != a.cloud_topic:
            continue
        m = deserialize_message(d, cloud_cls)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if T[0] <= st <= T[-1]:
            stamps.append(st)
    total = len(stamps)
    if total == 0:
        raise SystemExit('[에러] 궤적 시간 범위와 겹치는 점군이 없습니다. '
                         '두 bag 이 같은 원본인지 확인하세요.')
    half = total // 2
    print(f'    궤적 범위 내 점군 {total} 개 (전반 {half} / 후반 {total-half})')
    dropped = 0

    r = open_reader(a.cloud_bag)
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
        if n != a.cloud_topic:
            continue
        m = deserialize_message(d, cloud_cls)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st < T[0] or st > T[-1]:      # 궤적 범위 밖(초기화 전 등)은 버림
            dropped += 1
            continue
        # 이 점군 stamp 에 가장 가까운 궤적 자세
        i = int(np.searchsorted(T, st))
        i = max(0, min(i, len(T) - 1))
        if i > 0 and abs(T[i - 1] - st) < abs(T[i] - st):
            i -= 1
        # (옛 자세 → 새 자세) 회전 델타
        R = rot(TH2[i] - TH[i])
        # PointCloud2 필드 datatype 이 섞여 있으면 read_points_numpy 가 거부하므로
        # read_points 로 구조화 배열을 받아 x,y,z 만 float64 로 뽑는다.
        pts = point_cloud2.read_points(m, field_names=('x', 'y', 'z'),
                                       skip_nans=True)
        pts = np.asarray(pts)          # 구조화 배열 (x,y,z 필드)
        if pts.size == 0:
            arr = np.empty((0, 3))
        else:
            arr = np.stack([pts['x'], pts['y'], pts['z']],
                           axis=-1).astype(np.float64)
        arr = arr[np.isfinite(arr).all(axis=1)]
        if len(arr):
            # /cloud_registered 는 이미 world(옛 자세 기준) 좌표다.
            # 점의 '로봇 기준 상대 위치'는 그대로 두고, 로봇 자세만
            # 옛것(P_raw,TH) → 보정된것(P_cor,TH2) 으로 옮긴다.
            #   p_new = R_i (p_reg - P_i_raw) + P_i_cor,  R_i = rot(TH2_i - TH_i)
            rel = arr[:, :2] - P[i]                 # 로봇 기준 상대 위치
            xy = (R @ rel.T).T + P2[i]              # 보정된 자세로 옮김
            arr = np.column_stack([xy, arr[:, 2]])  # z 는 보정하지 않는다
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
    print(f'    범위 밖으로 버린 점군: {dropped} 개')

    print('\n[6] 전반/후반 겹침 측정 (루프 클로저 성공 시 첫바퀴≈둘째바퀴)...')
    res = o3d.pipelines.registration.registration_icp(
        B.voxel_down_sample(0.2), A.voxel_down_sample(0.2), 3.0, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    sh = float(np.linalg.norm(res.transformation[:3, 3]))
    yw = math.degrees(math.atan2(res.transformation[1, 0],
                                 res.transformation[0, 0]))
    print(f'    평행이동 {sh:.2f} m, 회전 {yw:+.2f} deg, RMSE {res.inlier_rmse:.3f} m')

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
