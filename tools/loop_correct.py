#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loop_correct.py — 루프 클로저 오차를 경로에 분배해 지도를 보정한다

  원리
    1. 초반 구간 지도와 후반 구간 점군을 ICP 로 맞춰 실제 누적 오차 T_lc 를 잰다.
       (끝점이 시작점과 같다고 가정하지 않는다. 다리 오도메트리의 시작-끝 차이는
        ICP 초기값으로만 쓴다)
    2. 각 프레임에 이동 거리 비율 a (0→1) 를 매기고, 보정량을 a 에 비례해 나눈다.
       T_a = exp(a * log(T_lc))  — SE(2) 지수/로그 사상
       후반을 통째로 미는 것이 아니라 전 구간에 고르게 흡수시킨다.
    3. 보정된 자세로 지도를 다시 쌓고, 전반/후반 겹침이 줄었는지 재측정한다.

  루프가 하나뿐이므로 제약도 하나다. 포즈 그래프 라이브러리가 필요 없다.

  사용: python3 loop_correct.py [bag] [voxel]
"""

import math
import os
import sys

import numpy as np
import yaml

EDGE_N = 700          # 초반/후반에서 쓸 프레임 수
ICP_MAX_CORR = 2.0    # 초기값이 좋으므로 좁게


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
    """4x4 동차행렬 → (theta, vx, vy). z 는 따로 다룬다."""
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


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def open_reader(bag):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=detect_storage(bag)),
           ConverterOptions('', ''))
    return r


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/data/bags/indoor/floor_0805_1720')
    voxel = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
    outdir = os.path.expanduser('~/fastlio_ws/results/odommap_corrected')
    os.makedirs(outdir, exist_ok=True)

    import open3d as o3d
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    CLOUD = '/utlidar/cloud_deskewed'
    ODOM = '/utlidar/robot_odom'

    r = open_reader(bag)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cloud_cls = get_message(types[CLOUD])
    odom_cls = get_message(types[ODOM])

    # ---------- 1) 오도메트리로 이동 거리 비율 만들기 ----------
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
    seg = np.linalg.norm(np.diff(oxy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    print(f'    {len(ot):,} 개,  총 이동 {s[-1]:.1f} m')

    # 다리 오도메트리 기준 시작-끝 차이 → ICP 초기값
    d_yaw = math.atan2(math.sin(oyaw[0] - oyaw[-1]), math.cos(oyaw[0] - oyaw[-1]))
    c, sn = math.cos(d_yaw), math.sin(d_yaw)
    R = np.array([[c, -sn], [sn, c]])
    t_init = oxy[0] - R @ oxy[-1]
    T_init = np.eye(4)
    T_init[:2, :2] = R
    T_init[0, 3], T_init[1, 3] = t_init
    print(f'    시작-끝 차이: {np.linalg.norm(oxy[-1]-oxy[0]):.2f} m, '
          f'{math.degrees(d_yaw):+.1f} deg  → ICP 초기값')

    # ---------- 2) 초반/후반 점군으로 실제 오차 측정 ----------
    print(f'\n[2] 초반/후반 {EDGE_N} 프레임 점군 수집...')
    r = open_reader(bag)
    total = 0
    while r.has_next():
        n, _, _ = r.read_next()
        if n == CLOUD:
            total += 1
    print(f'    총 {total} 프레임')

    r = open_reader(bag)
    early, late = [], []
    k = 0
    while r.has_next():
        n, d, _ = r.read_next()
        if n != CLOUD:
            continue
        if k < EDGE_N or k >= total - EDGE_N:
            m = deserialize_message(d, cloud_cls)
            a = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'),
                                              skip_nans=True)
            a = np.asarray(a, dtype=np.float64).reshape(-1, 3)
            a = a[np.isfinite(a).all(axis=1)]
            (early if k < EDGE_N else late).append(a)
        k += 1
    E = o3d.geometry.PointCloud()
    E.points = o3d.utility.Vector3dVector(np.vstack(early))
    E = E.voxel_down_sample(0.15)
    L = o3d.geometry.PointCloud()
    L.points = o3d.utility.Vector3dVector(np.vstack(late))
    L = L.voxel_down_sample(0.15)
    print(f'    초반 {len(E.points):,} 점,  후반 {len(L.points):,} 점')

    print('\n[3] ICP 로 실제 누적 오차 측정...')
    res = o3d.pipelines.registration.registration_icp(
        L, E, ICP_MAX_CORR, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
    T_lc = np.array(res.transformation)
    th, vx, vy = se2_log(T_lc)
    dz = float(T_lc[2, 3])
    print(f'    fitness {res.fitness:.3f},  RMSE {res.inlier_rmse:.3f} m')
    print(f'    누적 오차: 평행이동 {np.linalg.norm(T_lc[:2,3]):.2f} m, '
          f'회전 {math.degrees(th):+.2f} deg, z {dz:+.2f} m')

    if res.fitness < 0.3:
        print('\n    ⚠ fitness 가 낮습니다. 끝 구간이 시작 구역과 겹치지 않을 수 있습니다.')
        print('      보정을 진행해도 신뢰하기 어렵습니다.')

    # ---------- 3) 보정하며 지도 재구성 ----------
    print('\n[4] 보정 적용하며 지도 재구성...')
    r = open_reader(bag)
    A = o3d.geometry.PointCloud()   # 전반
    B = o3d.geometry.PointCloud()   # 후반
    bufA, bufB = [], []
    k = 0
    half = total // 2

    def flush(buf, pc):
        if not buf:
            return pc
        p = o3d.geometry.PointCloud()
        p.points = o3d.utility.Vector3dVector(np.vstack(buf))
        pc += p
        return pc.voxel_down_sample(voxel)

    while r.has_next():
        n, d, _ = r.read_next()
        if n != CLOUD:
            continue
        m = deserialize_message(d, cloud_cls)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        # 이 프레임까지의 이동 거리 비율
        i = int(np.searchsorted(ot, stamp))
        i = max(0, min(i, len(s) - 1))
        alpha = s[i] / s[-1] if s[-1] > 0 else 0.0

        T_a = se2_exp(alpha * th, alpha * vx, alpha * vy)
        T_a[2, 3] = 0.0   # z 는 보정하지 않는다 (다리 오도메트리가 이미 평평)

        a = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'),
                                           skip_nans=True)
        a = np.asarray(a, dtype=np.float64).reshape(-1, 3)
        a = a[np.isfinite(a).all(axis=1)]
        if len(a):
            a = (T_a[:3, :3] @ a.T).T + T_a[:3, 3]
            (bufA if k < half else bufB).append(a)
        k += 1
        if k % 400 == 0:
            if k <= half:
                A = flush(bufA, A); bufA = []
            else:
                B = flush(bufB, B); bufB = []
            print(f'    {k}/{total}  (a={alpha:.2f})')

    A = flush(bufA, A)
    B = flush(bufB, B)

    # ---------- 4) 보정 효과 검증 ----------
    print('\n[5] 보정 후 전반/후반 겹침 재측정...')
    SA = A.voxel_down_sample(0.2)
    SB = B.voxel_down_sample(0.2)
    res2 = o3d.pipelines.registration.registration_icp(
        SB, SA, 3.0, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    shift2 = float(np.linalg.norm(res2.transformation[:3, 3]))
    yaw2 = math.degrees(math.atan2(res2.transformation[1, 0],
                                   res2.transformation[0, 0]))

    print('\n### 보정 전후 비교\n')
    print('| 항목 | 보정 전 | 보정 후 |')
    print('|---|---|---|')
    print(f'| 전반-후반 평행이동 | 4.02 m | **{shift2:.2f} m** |')
    print(f'| 전반-후반 회전 | +7.85 deg | **{yaw2:+.2f} deg** |')
    print(f'| 정합 RMSE | 0.910 m | **{res2.inlier_rmse:.3f} m** |')

    M = A + B
    M = M.voxel_down_sample(voxel)
    pts = np.asarray(M.points)
    print(f'\n최종 지도 {len(pts):,} 점')
    print('\n| 항목 | 값 |')
    print('|---|---|')
    for i, ax in enumerate('xyz'):
        v = pts[:, i]
        print(f'| {ax} p1~p99 | {np.percentile(v,99)-np.percentile(v,1):.2f} m |')
    hist, edges = np.histogram(pts[:, 2], bins=100)
    kk = int(np.argmax(hist))
    print(f'| 지면 추정 z | {(edges[kk]+edges[kk+1])/2:.3f} m |')

    out = os.path.join(outdir, 'scans.pcd')
    o3d.io.write_point_cloud(out, M)
    print(f'\n저장: {out}')
    print('\n다음:')
    print(f'  python3 ~/fastlio_ws/tools/pcd_to_grid.py \\')
    print(f'    {out} \\')
    print(f'    {outdir}/grid 0.10')


if __name__ == '__main__':
    main()
