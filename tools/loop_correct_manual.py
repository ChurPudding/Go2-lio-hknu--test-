#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loop_correct_manual.py — 실측 사실을 제약으로 넣어 보정한다

  기존 loop_correct.py 는 오차를 ICP 로 '추정'했다.
  0807 bag 에서 fitness 0.665 로 실패한 지점이 거기다.

  그런데 사람은 답을 안다: 로봇이 출발점으로 돌아왔다는 사실 자체가
  가장 강한 제약이다. 추정하지 말고 그냥 넣는다.

      T_lc = X_true · X_measured^-1        (끝 자세를 진짜 값으로 강제)
      T_a  = exp(a · log(T_lc))            (이동거리 비율로 분배)

  기본값은 "출발점에 정확히, 같은 방향으로 복귀".
  실제와 다르면 --end-dx / --end-dy / --end-dyaw 로 알려주면 된다.
  (시작 자세 기준 상대값. 예: 반대 방향으로 끝났으면 --end-dyaw 180)

  사용 예:
    # 출발점에 정확히, 반대 방향으로 끝남
    python3 loop_correct_manual.py ~/data/bags/indoor/floor_0807_1542 --end-dyaw 180

    # 출발점에서 앞으로 1 m 지점, 반대 방향
    python3 loop_correct_manual.py ~/data/bags/indoor/floor_0807_1542 \
        --end-dx 1.0 --end-dyaw 180

    # 궤적만 그려보고 끝내기 (지도 생성 안 함)
    python3 loop_correct_manual.py ~/data/bags/indoor/floor_0807_1542 --plot-only
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


# ---------------- SE(2) 지수/로그 ----------------
def se2_log(T):
    th = math.atan2(T[1, 0], T[0, 0])
    t = np.array([T[0, 3], T[1, 3]])
    if abs(th) < 1e-9:
        return th, t[0], t[1]
    A = math.sin(th) / th
    B = (1 - math.cos(th)) / th
    V = np.array([[A, -B], [B, A]])
    v = np.linalg.solve(V, t)
    return th, v[0], v[1]


def se2_exp(th, vx, vy):
    T = np.eye(4)
    c, s = math.cos(th), math.sin(th)
    T[0, 0], T[0, 1] = c, -s
    T[1, 0], T[1, 1] = s, c
    if abs(th) < 1e-9:
        T[0, 3], T[1, 3] = vx, vy
    else:
        A = math.sin(th) / th
        B = (1 - math.cos(th)) / th
        V = np.array([[A, -B], [B, A]])
        t = V @ np.array([vx, vy])
        T[0, 3], T[1, 3] = t[0], t[1]
    return T


def se2_mat(x, y, th):
    T = np.eye(4)
    c, s = math.cos(th), math.sin(th)
    T[0, 0], T[0, 1] = c, -s
    T[1, 0], T[1, 1] = s, c
    T[0, 3], T[1, 3] = x, y
    return T


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def open_reader(bag):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=detect_storage(bag)),
           ConverterOptions('', ''))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', nargs='?',
                    default=os.path.expanduser('~/data/bags/indoor/floor_0807_1542'))
    ap.add_argument('--voxel', type=float, default=0.05)
    ap.add_argument('--end-dx', type=float, default=0.0,
                    help='실제 끝 위치 (시작 자세 기준 전방, m)')
    ap.add_argument('--end-dy', type=float, default=0.0,
                    help='실제 끝 위치 (시작 자세 기준 좌측, m)')
    ap.add_argument('--end-dyaw', type=float, default=0.0,
                    help='실제 끝 방향 (시작 대비, deg). 반대면 180')
    ap.add_argument('--out', default=os.path.expanduser(
        '~/fastlio_ws/results/odommap_manual'))
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

    # ---------- 1) 오도메트리 ----------
    print('[1] 오도메트리 읽는 중...')
    ot, oxy, oyaw = [], [], []
    while r.has_next():
        n, d, _ = r.read_next()
        if n != ODOM:
            continue
        m = deserialize_message(d, odom_cls)
        p = m.pose.pose.position
        ot.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        oxy.append((p.x, p.y))
        oyaw.append(yaw_of(m.pose.pose.orientation))
    ot = np.array(ot)
    oxy = np.array(oxy)
    oyaw = np.array(oyaw)
    seg = np.linalg.norm(np.diff(oxy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])

    # 누적 절대 yaw (lever arm 검증용)
    dy = np.diff(oyaw)
    dy = np.arctan2(np.sin(dy), np.cos(dy))
    cum_yaw = float(np.abs(dy).sum())

    print(f'    {len(ot):,} 개,  총 이동 {s[-1]:.1f} m')
    print(f'    누적 절대 yaw {math.degrees(cum_yaw):,.0f} deg  '
          f'→ lever 0.322 m 로 인한 경로 증가 {0.322*cum_yaw:.1f} m')

    # ---------- 2) 측정 vs 실측 ----------
    th0 = oyaw[0]
    X0 = se2_mat(oxy[0, 0], oxy[0, 1], th0)
    XN = se2_mat(oxy[-1, 0], oxy[-1, 1], oyaw[-1])

    # 실제 끝 자세 = 시작 자세 · (dx, dy, dyaw)
    Xtrue = X0 @ se2_mat(a.end_dx, a.end_dy, math.radians(a.end_dyaw))

    meas_d = float(np.linalg.norm(oxy[-1] - oxy[0]))
    meas_yaw = math.degrees(wrap(oyaw[-1] - th0))

    print('\n[2] 측정값 vs 실측 제약\n')
    print('| 항목 | 오도메트리 측정 | 실측(입력) | 차이 = 보정량 |')
    print('|---|---|---|---|')
    print(f'| 끝 위치 (시작 기준) | {meas_d:.2f} m | '
          f'{math.hypot(a.end_dx, a.end_dy):.2f} m | — |')
    print(f'| 끝 방향 (시작 대비) | {meas_yaw:+.1f}° | {a.end_dyaw:+.1f}° | '
          f'{wrap(math.radians(a.end_dyaw) - wrap(oyaw[-1]-th0))*180/math.pi:+.1f}° |')

    T_lc = Xtrue @ np.linalg.inv(XN)
    th, vx, vy = se2_log(T_lc)
    shift = float(np.linalg.norm(T_lc[:2, 3]))
    print(f'\n    보정 변환: 평행이동 {shift:.2f} m, 회전 {math.degrees(th):+.2f}°')
    if shift > 15.0 or abs(math.degrees(th)) > 60.0:
        print('    ⚠ 보정량이 큽니다. --end-* 입력값을 다시 확인하세요.')

    # ---------- 3) 궤적 그림 ----------
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        corr = np.zeros_like(oxy)
        for i in range(len(oxy)):
            al = s[i] / s[-1] if s[-1] > 0 else 0.0
            Ta = se2_exp(al * th, al * vx, al * vy)
            corr[i] = Ta[:2, :2] @ oxy[i] + Ta[:2, 3]

        fig, ax = plt.subplots(1, 2, figsize=(15, 7))
        sc = ax[0].scatter(oxy[:, 0], oxy[:, 1], c=ot - ot[0], s=1, cmap='viridis')
        ax[0].plot(oxy[0, 0], oxy[0, 1], 'go', ms=12, label='start')
        ax[0].plot(oxy[-1, 0], oxy[-1, 1], 'rx', ms=14, mew=3, label='end')
        ax[0].set_title(f'leg odometry (raw)   start-end {meas_d:.2f} m')
        plt.colorbar(sc, ax=ax[0], label='elapsed [s]')

        ax[1].plot(oxy[:, 0], oxy[:, 1], lw=1, c='tab:red', alpha=.6, label='raw')
        ax[1].plot(corr[:, 0], corr[:, 1], lw=1, c='tab:blue', label='corrected')
        ax[1].plot(corr[0, 0], corr[0, 1], 'go', ms=10)
        ax[1].plot(corr[-1, 0], corr[-1, 1], 'bx', ms=12, mew=3)
        ax[1].set_title('raw vs corrected')
        ax[1].legend()

        for x in ax:
            x.set_aspect('equal'); x.grid(alpha=.3)
            x.set_xlabel('x [m]'); x.set_ylabel('y [m]')
        ax[0].legend()
        plt.tight_layout()
        p = os.path.join(a.out, 'trajectory.png')
        plt.savefig(p, dpi=115); plt.close()
        print(f'\n[3] 궤적 그림: {p}')
        print('    ★ 실제로 걸으신 경로와 모양이 맞는지 확인해 주세요.')
    except Exception as e:
        print(f'\n[3] (그림 생략: {e})')

    if a.plot_only:
        print('\n--plot-only 지정. 지도 생성은 건너뜁니다.')
        return

    # ---------- 4) 보정하며 지도 재구성 ----------
    import open3d as o3d
    print('\n[4] 보정 적용하며 지도 재구성...')
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
        p = o3d.geometry.PointCloud()
        p.points = o3d.utility.Vector3dVector(np.vstack(buf))
        pc += p
        return pc.voxel_down_sample(a.voxel)

    while r.has_next():
        n, d, _ = r.read_next()
        if n != CLOUD:
            continue
        m = deserialize_message(d, cloud_cls)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        i = int(np.searchsorted(ot, stamp))
        i = max(0, min(i, len(s) - 1))
        alpha = s[i] / s[-1] if s[-1] > 0 else 0.0
        Ta = se2_exp(alpha * th, alpha * vx, alpha * vy)   # z 는 보정하지 않는다

        arr = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'),
                                             skip_nans=True)
        arr = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
        arr = arr[np.isfinite(arr).all(axis=1)]
        if len(arr):
            arr = (Ta[:3, :3] @ arr.T).T + Ta[:3, 3]
            (bufA if k < half else bufB).append(arr)
        k += 1
        if k % 400 == 0:
            if k <= half:
                A = flush(bufA, A); bufA = []
            else:
                B = flush(bufB, B); bufB = []
            print(f'    {k}/{total}  (a={alpha:.2f})')

    A = flush(bufA, A)
    B = flush(bufB, B)

    # ---------- 5) 겹침 재측정 ----------
    print('\n[5] 전반/후반 겹침 측정...')
    SA = A.voxel_down_sample(0.2)
    SB = B.voxel_down_sample(0.2)
    res = o3d.pipelines.registration.registration_icp(
        SB, SA, 3.0, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    sh = float(np.linalg.norm(res.transformation[:3, 3]))
    yw = math.degrees(math.atan2(res.transformation[1, 0],
                                 res.transformation[0, 0]))
    print(f'    평행이동 {sh:.2f} m, 회전 {yw:+.2f}°, RMSE {res.inlier_rmse:.3f} m')

    M = (A + B).voxel_down_sample(a.voxel)
    pts = np.asarray(M.points)
    print(f'\n최종 지도 {len(pts):,} 점')
    print('\n| 항목 | 값 |')
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
