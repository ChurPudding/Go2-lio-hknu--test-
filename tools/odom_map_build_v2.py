#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_map_build_v2.py — 축척 보정을 넣어 점군을 누적한다

  odom_map_build.py 와 같은 일을 하되, 다리 오도메트리의 축척 부족을 보정한다.

  원리
    cloud_deskewed 의 점은 이미 odom 좌표계이므로
        p_odom(t) = R(t)·p_body + t(t)
    축척 오차는 로봇 위치 t(t) 에만 있고 R(t) 와 p_body 는 참값이다. 따라서
        p'(t) = p_odom(t) + (k-1)·t(t)
    로 각 프레임을 밀어주면 궤적만 k 배로 늘어나고 스캔 자체는 그대로 남는다.
    좌표 변환도 레버암도 필요 없다.

  근거 (2026-08-10 실측)
    줄자·타일 51.83 m  vs  다리 오도메트리 43.21 m  →  k = 1.1995
    6회 반복, 편차 0.33 m. 전진·후진 차이 0.37% 로 방향 의존성 없음.
    지도상 가로·세로 부족률이 13.1% / 13.19% 로 일치 → 등방 축소 확인.

  사용: python3 odom_map_build_v2.py [bag] [voxel] [출력] [--k 1.1995]
  예:   python3 odom_map_build_v2.py ~/data/bags/indoor/floor_0805_1720 0.05 \
            ~/fastlio_ws/results/odommap_v3/scans.pcd
"""

import argparse
import os

import numpy as np
import yaml


CLOUD_TOPIC = '/utlidar/cloud_deskewed'
ODOM_TOPIC = '/utlidar/robot_odom'


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


def open_reader(bag, storage):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=storage), ConverterOptions('', ''))
    return r


def read_trajectory(bag, storage, types):
    """1차 통과: 로봇 위치를 시각과 함께 모은다. bag 수신 시각을 쓴다."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    cls = get_message(types[ODOM_TOPIC])
    reader = open_reader(bag, storage)
    from rosbag2_py import StorageFilter
    reader.set_filter(StorageFilter(topics=[ODOM_TOPIC]))

    t, xyz = [], []
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        m = deserialize_message(data, cls)
        p = m.pose.pose.position
        t.append(t_ns * 1e-9)
        xyz.append((p.x, p.y, p.z))

    t = np.asarray(t)
    xyz = np.asarray(xyz)
    order = np.argsort(t)
    return t[order], xyz[order]


def path_length(xyz):
    d = np.diff(xyz[:, :2], axis=0)
    return float(np.hypot(d[:, 0], d[:, 1]).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', nargs='?',
                    default=os.path.expanduser('~/data/bags/indoor/floor_0805_1720'))
    ap.add_argument('voxel', nargs='?', type=float, default=0.05)
    ap.add_argument('out', nargs='?',
                    default=os.path.expanduser('~/fastlio_ws/results/odommap_v3/scans.pcd'))
    ap.add_argument('--k', type=float, default=1.1995,
                    help='축척 보정계수. 1.0 이면 보정 없음(기존과 동일)')
    args = ap.parse_args()

    import open3d as o3d
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2
    from rosbag2_py import StorageFilter

    bag, voxel, out, k = args.bag, args.voxel, args.out, args.k
    os.makedirs(os.path.dirname(out), exist_ok=True)
    storage = detect_storage(bag)

    types = {t.name: t.type for t in open_reader(bag, storage).get_all_topics_and_types()}
    for need in (CLOUD_TOPIC, ODOM_TOPIC):
        if need not in types:
            print(f'{need} 가 bag 에 없습니다.')
            return

    print(f'bag    : {bag}')
    print(f'voxel  : {voxel} m')
    print(f'k      : {k}\n')

    ot, oxyz = read_trajectory(bag, storage, types)
    L = path_length(oxyz)
    print(f'궤적    : {len(ot):,} 샘플 · 경로장 {L:.2f} m → 보정 후 {L*k:.2f} m')
    span = np.hypot(oxyz[:, 0].ptp(), oxyz[:, 1].ptp())
    print(f'          xy 범위 {span:.2f} m → 보정 후 {span*k:.2f} m\n')

    msg_cls = get_message(types[CLOUD_TOPIC])
    reader = open_reader(bag, storage)
    reader.set_filter(StorageFilter(topics=[CLOUD_TOPIC]))

    acc = o3d.geometry.PointCloud()
    buf = []
    n_msg = 0
    frame_ids = set()
    raw_total = 0
    n_outside = 0

    def flush():
        nonlocal acc, buf
        if not buf:
            return
        pts = np.vstack(buf)
        buf.clear()
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pts)
        acc += pc
        acc = acc.voxel_down_sample(voxel)

    while reader.has_next():
        _, data, t_ns = reader.read_next()
        m = deserialize_message(data, msg_cls)
        frame_ids.add(m.header.frame_id)

        arr = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'),
                                             skip_nans=True)
        arr = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
        arr = arr[np.isfinite(arr).all(axis=1)]
        if not len(arr):
            n_msg += 1
            continue

        t = t_ns * 1e-9
        if t < ot[0] or t > ot[-1]:
            n_outside += 1
        pos = np.array([np.interp(t, ot, oxyz[:, i]) for i in range(3)])

        arr = arr + (k - 1.0) * pos          # 궤적만 k 배로 늘린다

        buf.append(arr)
        raw_total += len(arr)
        n_msg += 1
        if n_msg % 200 == 0:
            flush()
            print(f'  {n_msg} 프레임  누적 {len(acc.points):,} 점')

    flush()
    pts = np.asarray(acc.points)

    print(f'\n프레임 수    : {n_msg:,}')
    print(f'frame_id     : {sorted(frame_ids)}')
    print(f'원시 점 합계 : {raw_total:,}')
    print(f'다운샘플 후  : {len(pts):,}')
    if n_outside:
        print(f'! 궤적 시간 밖 프레임 {n_outside} 개 — 끝단 위치로 외삽했습니다')
    if not (frame_ids & {'odom', 'map', 'odom_frame'}):
        print(f'! frame_id 가 odom 계열이 아닙니다. 점군이 이미 odom 좌표인지 확인하십시오.')
    print()

    if len(pts) == 0:
        print('점이 없습니다.')
        return

    print('### 지도 지표\n')
    print('| 항목 | 값 |')
    print('|---|---|')
    for i, ax in enumerate('xyz'):
        v = pts[:, i]
        print(f'| {ax} p1~p99 | {np.percentile(v,99)-np.percentile(v,1):.2f} m |')
    for i, ax in enumerate('xyz'):
        v = pts[:, i]
        print(f'| {ax} min~max | {v.max()-v.min():.2f} m |')
    hist, edges = np.histogram(pts[:, 2], bins=100)
    j = int(np.argmax(hist))
    print(f'| 지면 추정 z | {(edges[j]+edges[j+1])/2:.3f} m |')

    print('\n### 높이 분포 상위 8\n')
    order = np.argsort(hist)[::-1][:8]
    for j in sorted(order):
        bar = '#' * int(50 * hist[j] / hist.max())
        print(f'  {edges[j]:+6.2f} ~ {edges[j+1]:+6.2f} m  {bar}')

    o3d.io.write_point_cloud(out, acc)
    print(f'\n저장: {out}  ({os.path.getsize(out)/1e6:.1f} MB)')
    print('\n다음:')
    print(f'  python3 ~/fastlio_ws/tools/pcd_to_grid.py \\')
    print(f'    {out} \\')
    print(f'    {os.path.join(os.path.dirname(out), "grid")} 0.10')


if __name__ == '__main__':
    main()
