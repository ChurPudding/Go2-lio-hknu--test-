#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_split_check.py — 왕복 구간이 겹쳐 그려졌는지 판정한다

  같은 복도를 두 번 지나면, 드리프트가 있을 때 두 번째 통과가 어긋난 위치에
  새로 그려진다. 눈으로는 "벽이 여러 줄"로 보여 벽인지 겹침인지 구분이 안 된다.

  그래서 프레임을 전반/후반으로 나눠 각각 지도를 만들고 겹쳐 본다.
    · 두 색이 포개진다   → 겹침 없음. 지도 정상
    · 두 색이 어긋난다   → 겹침. 어긋난 거리를 ICP 로 잰다

  사용: python3 map_split_check.py [bag] [voxel]
"""

import os
import sys

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


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/data/bags/indoor/floor_0805_1720')
    voxel = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
    outdir = os.path.expanduser('~/fastlio_ws/results/odommap')
    os.makedirs(outdir, exist_ok=True)

    import open3d as o3d
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    TOPIC = '/utlidar/cloud_deskewed'

    # 1) 전체 프레임 수 파악
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=detect_storage(bag)),
                ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = get_message(types[TOPIC])
    total = sum(1 for _ in iter(lambda: reader.read_next() if reader.has_next() else None,
                                None) if _ and _[0] == TOPIC)
    half = total // 2
    print(f'총 {total} 프레임 → 전반 0~{half}, 후반 {half}~{total}\n')

    # 2) 두 번에 나눠 누적
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=detect_storage(bag)),
                ConverterOptions('', ''))

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
        return pc.voxel_down_sample(voxel)

    while reader.has_next():
        tname, data, _ = reader.read_next()
        if tname != TOPIC:
            continue
        m = deserialize_message(data, msg_cls)
        arr = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'),
                                             skip_nans=True)
        arr = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
        arr = arr[np.isfinite(arr).all(axis=1)]
        if len(arr):
            (bufA if k < half else bufB).append(arr)
        k += 1
        if k % 200 == 0:
            if k <= half:
                A = flush(bufA, A); bufA = []
            else:
                B = flush(bufB, B); bufB = []
            print(f'  {k} 프레임  전반 {len(A.points):,}  후반 {len(B.points):,}')

    A = flush(bufA, A)
    B = flush(bufB, B)
    pa = np.asarray(A.points)
    pb = np.asarray(B.points)
    print(f'\n전반 {len(pa):,} 점,  후반 {len(pb):,} 점')

    # 3) 겹치는 영역만 남기고 ICP
    print('\n### 두 절반이 얼마나 어긋났는가\n')
    lo = np.maximum(pa.min(0), pb.min(0))
    hi = np.minimum(pa.max(0), pb.max(0))
    ma = ((pa >= lo) & (pa <= hi)).all(1)
    mb = ((pb >= lo) & (pb <= hi)).all(1)
    print(f'겹치는 영역: x {lo[0]:.1f}~{hi[0]:.1f}, y {lo[1]:.1f}~{hi[1]:.1f} m')
    print(f'그 안의 점: 전반 {ma.sum():,},  후반 {mb.sum():,}')

    if ma.sum() > 1000 and mb.sum() > 1000:
        SA = o3d.geometry.PointCloud()
        SA.points = o3d.utility.Vector3dVector(pa[ma])
        SB = o3d.geometry.PointCloud()
        SB.points = o3d.utility.Vector3dVector(pb[mb])
        SA = SA.voxel_down_sample(0.2)
        SB = SB.voxel_down_sample(0.2)

        res = o3d.pipelines.registration.registration_icp(
            SB, SA, 3.0, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
        T = res.transformation
        shift = float(np.linalg.norm(T[:3, 3]))
        yaw = float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))

        print('\n| 항목 | 값 |')
        print('|---|---|')
        print(f'| 정합 전 대응 비율 (fitness) | {res.fitness:.3f} |')
        print(f'| **후반을 전반에 맞추는 평행이동** | **{shift:.2f} m** |')
        print(f'| 후반을 전반에 맞추는 회전 | {yaw:+.2f} deg |')
        print(f'| 정합 후 RMSE | {res.inlier_rmse:.3f} m |')

        print('\n### 판정\n')
        if shift < 0.5:
            print(f'· 어긋남 {shift:.2f} m — 겹침 없음. 왕복 구간이 정상 정합됐다.')
            print('  → 실내 지도 요구사항 충족. A* 에 넘길 수 있다.')
        elif shift < 2.0:
            print(f'· 어긋남 {shift:.2f} m — 경미. 복도 폭보다 작으면 실용상 문제 없음.')
            print('  → 격자 해상도 0.10 m 기준 벽이 다소 두꺼워지는 정도.')
        else:
            print(f'· 어긋남 {shift:.2f} m — 겹침 있음.')
            print('  → 루프 클로저 또는 다리 오도메트리 보정이 필요.')

    # 4) 그림
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1, 2, figsize=(16, 7))
        for a in ax:
            a.set_aspect('equal'); a.grid(alpha=.3)
            a.set_xlabel('x [m]'); a.set_ylabel('y [m]')

        s = max(1, len(pa) // 400000)
        ax[0].scatter(pa[::s, 0], pa[::s, 1], s=.12, c='tab:blue',
                      label=f'front half (0~{half})', alpha=.5, linewidths=0)
        s = max(1, len(pb) // 400000)
        ax[0].scatter(pb[::s, 0], pb[::s, 1], s=.12, c='tab:red',
                      label=f'back half ({half}~{k})', alpha=.5, linewidths=0)
        ax[0].legend(markerscale=30)
        ax[0].set_title('front vs back half — overlap check')

        # 복도 구간 확대
        sel_a = pa[(pa[:, 0] > 20) & (pa[:, 0] < 50)]
        sel_b = pb[(pb[:, 0] > 20) & (pb[:, 0] < 50)]
        if len(sel_a):
            ax[1].scatter(sel_a[:, 0], sel_a[:, 1], s=.3, c='tab:blue',
                          alpha=.5, linewidths=0)
        if len(sel_b):
            ax[1].scatter(sel_b[:, 0], sel_b[:, 1], s=.3, c='tab:red',
                          alpha=.5, linewidths=0)
        ax[1].set_title('corridor x=20~50 (zoom)')

        plt.tight_layout()
        out = os.path.join(outdir, 'split_check.png')
        plt.savefig(out, dpi=110)
        print(f'\n그림: {out}')
    except Exception as e:
        print(f'\n(그림 생략: {e})')


if __name__ == '__main__':
    main()
