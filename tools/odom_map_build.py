#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_map_build.py — 다리 오도메트리 자세로 점군을 누적해 지도를 만든다

  근거: 같은 bag 에서 다리 오도메트리 드리프트 2.63% vs Point-LIO 21.75%.
        /utlidar/cloud_deskewed 는 이미 odom 좌표계이므로 변환 없이 누적 가능.

  LIO 를 거치지 않고 A* 용 지도가 나오는지 확인하는 것이 목적이다.
  bag 재생 없이 오프라인으로 직접 읽는다.

  사용: python3 odom_map_build.py [bag] [voxel] [출력]
  예:   python3 odom_map_build.py ~/data/bags/indoor/floor_0805_1720 0.05
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
    out = (sys.argv[3] if len(sys.argv) > 3
           else os.path.expanduser('~/fastlio_ws/results/odommap/scans.pcd'))

    import open3d as o3d
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    os.makedirs(os.path.dirname(out), exist_ok=True)
    storage = detect_storage(bag)

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=storage), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    TOPIC = '/utlidar/cloud_deskewed'
    if TOPIC not in types:
        print(f'{TOPIC} 가 bag 에 없습니다. 들어 있는 점군 토픽:')
        for n, t in types.items():
            if 'PointCloud2' in t:
                print(f'  {n}')
        print('\n→ cloud_deskewed 가 없으면 이 방법은 쓸 수 없습니다.')
        return

    msg_cls = get_message(types[TOPIC])
    print(f'bag    : {bag}')
    print(f'topic  : {TOPIC}')
    print(f'voxel  : {voxel} m\n')

    acc = o3d.geometry.PointCloud()
    buf = []
    n_msg = 0
    frame_ids = set()
    raw_total = 0

    def flush():
        nonlocal acc, buf
        if not buf:
            return
        pts = np.vstack(buf)
        buf = []
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pts)
        acc += pc
        acc = acc.voxel_down_sample(voxel)

    while reader.has_next():
        tname, data, _ = reader.read_next()
        if tname != TOPIC:
            continue
        m = deserialize_message(data, msg_cls)
        frame_ids.add(m.header.frame_id)
        arr = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'),
                                             skip_nans=True)
        arr = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
        arr = arr[np.isfinite(arr).all(axis=1)]
        if len(arr):
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
    print(f'다운샘플 후  : {len(pts):,}\n')

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
    k = int(np.argmax(hist))
    print(f'| 지면 추정 z | {(edges[k]+edges[k+1])/2:.3f} m |')

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
    print(f'    ~/fastlio_ws/results/odommap/grid 0.10')


if __name__ == '__main__':
    main()
