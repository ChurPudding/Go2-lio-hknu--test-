#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
legodom_check.py — bag 의 다리 오도메트리 드리프트를 잰다

  Point-LIO 는 같은 bag 에서 시작-끝 거리 57 m (경로 260 m 의 22%) 를 냈다.
  실내 LIO 통상값은 1~2% 이므로 정상 범위 밖이다.

  bag 에 /utlidar/robot_odom 이 이미 들어 있으므로, 재생 없이 같은 지표를 잰다.
  두 값을 비교해서 다음 순서를 정한다:
    · 다리 오도메트리가 훨씬 낫다 → 융합/초기값 제공이 루프 클로저보다 먼저
    · 비슷하게 나쁘다             → 루프 클로저로 진행

  사용: python3 legodom_check.py [bag 경로] [토픽]
"""

import math
import os
import sys

import numpy as np
import yaml

TOPIC_DEFAULT = '/utlidar/robot_odom'


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


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/data/bags/indoor/floor_0805_1720')
    topic = sys.argv[2] if len(sys.argv) > 2 else TOPIC_DEFAULT

    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage = detect_storage(bag)
    print(f'bag     : {bag}')
    print(f'storage : {storage}')
    print(f'topic   : {topic}\n')

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=storage),
                ConverterOptions('', ''))

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        print('해당 토픽이 bag 에 없습니다. 들어 있는 odom 계열:')
        for n, t in types.items():
            if 'odom' in n.lower() or 'Odometry' in t:
                print(f'  {n}  ({t})')
        return

    msg_cls = get_message(types[topic])
    print(f'타입    : {types[topic]}\n')

    T, P, Y = [], [], []
    while reader.has_next():
        tname, data, tstamp = reader.read_next()
        if tname != topic:
            continue
        m = deserialize_message(data, msg_cls)
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        P.append((p.x, p.y, p.z))
        Y.append(yaw_from_quat(q.x, q.y, q.z, q.w))

    if len(P) < 2:
        print('메시지가 부족합니다.')
        return

    T = np.array(T)
    P = np.array(P)
    Y = np.array(Y)

    d = np.diff(P, axis=0)
    path = float(np.linalg.norm(d, axis=1).sum())
    path_xy = float(np.linalg.norm(d[:, :2], axis=1).sum())
    se = float(np.linalg.norm(P[-1] - P[0]))
    se_xy = float(np.linalg.norm(P[-1, :2] - P[0, :2]))
    dur = float(T[-1] - T[0])
    dyaw = math.degrees(math.atan2(math.sin(Y[-1] - Y[0]), math.cos(Y[-1] - Y[0])))

    print('### 다리 오도메트리 (/utlidar/robot_odom)\n')
    print('| 항목 | 값 |')
    print('|---|---|')
    print(f'| 메시지 수 | {len(P):,} |')
    print(f'| 구간 길이 | {dur:.1f} s |')
    print(f'| 경로 길이 (3D) | {path:.1f} m |')
    print(f'| 경로 길이 (xy) | {path_xy:.1f} m |')
    print(f'| **시작-끝 거리 (xy)** | **{se_xy:.2f} m** |')
    print(f'| 시작-끝 거리 (3D) | {se:.2f} m |')
    print(f'| 시작-끝 yaw 차 | {dyaw:+.1f} deg |')
    print(f'| **드리프트율** | **{100*se_xy/max(path_xy,1e-9):.2f} %** |')
    print(f'| z 범위 | {P[:,2].min():.2f} ~ {P[:,2].max():.2f} m |')

    print('\n### Point-LIO 와 비교 (같은 bag)\n')
    LIO_SE, LIO_PATH = 56.93, 261.76     # base_1~3 평균
    print('| | 다리 오도메트리 | Point-LIO |')
    print('|---|---|---|')
    print(f'| 경로 길이 (xy) | {path_xy:.1f} m | {LIO_PATH:.1f} m |')
    print(f'| 시작-끝 거리 | **{se_xy:.2f} m** | 56.93 m |')
    print(f'| 드리프트율 | **{100*se_xy/max(path_xy,1e-9):.2f} %** '
          f'| {100*LIO_SE/LIO_PATH:.2f} % |')

    r = se_xy / LIO_SE if LIO_SE else float('inf')
    print('\n### 판정\n')
    if r < 0.3:
        print(f'· 다리 오도메트리가 {1/r:.0f}배 정확하다.')
        print('  → 루프 클로저보다 먼저 다리 오도메트리 활용을 검토할 것.')
        print('    (Point-LIO 초기값 제공 또는 팩터 그래프에서 odometry factor)')
    elif r > 3.0:
        print('· 다리 오도메트리가 더 나쁘다. Point-LIO 프론트엔드 유지.')
        print('  → 루프 클로저로 진행.')
    else:
        print('· 두 방식이 비슷한 수준이다.')
        print('  → 어느 쪽도 단독으로는 부족. 융합 또는 루프 클로저 필요.')

    out = os.path.expanduser('~/fastlio_ws/results/repro/legodom_traj.csv')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write('stamp,x,y,z,yaw_deg\n')
        for i in range(len(P)):
            f.write('%.6f,%.6f,%.6f,%.6f,%.3f\n'
                    % (T[i], P[i, 0], P[i, 1], P[i, 2], math.degrees(Y[i])))
    print(f'\n궤적 저장: {out}')


if __name__ == '__main__':
    main()
