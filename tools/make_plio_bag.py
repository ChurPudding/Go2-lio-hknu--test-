#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_plio_bag.py — Point-LIO 재생용 bag 을 미리 만든다

  왜 필요한가
  ----------
  /sportmodestate 는 재생 시 발행이 실패한다.
      failed to publish serialized message: cannot publish data
  녹화 당시 unitree_go 와 현재 설치된 unitree_go 의 메시지 정의가 다르기 때문이다
  (bag 안 516 byte vs 설치된 정의로 직렬화하면 그보다 작다 — path_point 유무).
  rosbag2 는 녹화된 바이트를 그대로 내보내므로 DDS 가 거부한다.
  파이썬으로 읽는 것은 정상이다. CDR 은 앞에서부터 필요한 만큼만 읽고 나머지를
  무시하므로, accelerometer 처럼 앞쪽에 있는 필드는 값이 정확하다
  (2026-08-10 확인: /lowstate 와 |a| 평균 9.4960 vs 9.4947 로 일치).

  그래서 l1_imu_fix.py 가 실시간으로 하던 일을 오프라인에서 미리 해두고,
  재생할 때는 /sportmodestate 를 아예 쓰지 않는다.

  부수 효과
  --------
  - 재생할 bag 이 가벼워진다 (cloud + imu 만)
  - 브리지 노드가 필요 없다
  - 타임스탬프로 짝을 맞추므로 매 실행 입력이 동일하다.
    실시간 브리지는 DDS 도착 순서에 따라 미세하게 달라졌다.

  사용:
    python3 make_plio_bag.py <입력bag> <출력bag> [--acc-topic /sportmodestate]
  예:
    python3 make_plio_bag.py ~/data/bags/indoor/loop_0810_1 \
                             ~/data/bags/indoor/loop_0810_1_plio
"""

import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from go2_calib import R_LB, ACC_SCALE_BODY  # noqa: E402

CLOUD = '/utlidar/cloud'
IMU = '/utlidar/imu'
OUT_IMU = '/l1_imu_fixed'


def detect_storage(bag_dir):
    meta = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.exists(meta):
        with open(meta) as f:
            m = yaml.safe_load(f)
        try:
            return m['rosbag2_bagfile_information']['storage_identifier']
        except Exception:
            pass
    return 'sqlite3'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--acc-topic', default='/sportmodestate',
                    help='가속도 출처. /sportmodestate 또는 /lowstate')
    a = ap.parse_args()

    from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                            ConverterOptions, StorageFilter, TopicMetadata)
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import Imu

    src = os.path.expanduser(a.src)
    dst = os.path.expanduser(a.dst)
    if os.path.exists(dst):
        sys.exit(f'출력 경로가 이미 있습니다: {dst}')

    storage = detect_storage(src)

    def reader(topics=None):
        r = SequentialReader()
        r.open(StorageOptions(uri=src, storage_id=storage), ConverterOptions('', ''))
        if topics:
            r.set_filter(StorageFilter(topics=topics))
        return r

    types = {t.name: t.type for t in reader().get_all_topics_and_types()}
    for need in (CLOUD, IMU, a.acc_topic):
        if need not in types:
            sys.exit(f'{need} 가 bag 에 없습니다. 담긴 토픽: {sorted(types)}')

    # ---------- 1) 가속도 수집 ----------
    print(f'[1] {a.acc_topic} 에서 가속도 읽는 중...')
    acc_cls = get_message(types[a.acc_topic])
    r = reader([a.acc_topic])
    at, av = [], []
    while r.has_next():
        _, d, t_ns = r.read_next()
        m = deserialize_message(d, acc_cls)
        v = m.imu_state.accelerometer
        at.append(t_ns)
        av.append((float(v[0]), float(v[1]), float(v[2])))
    at = np.asarray(at, dtype=np.int64)
    av = np.asarray(av, dtype=float)
    k = np.argsort(at)
    at, av = at[k], av[k]
    print(f'    {len(at):,} 개')
    print(f'    |a| 평균 {np.linalg.norm(av, axis=1).mean():.4f} '
          f'(정지 기준 9.465 근처여야 정상)')

    # ---------- 2) 변환하며 새 bag 작성 ----------
    print(f'[2] 새 bag 작성: {dst}')
    w = SequentialWriter()
    w.open(StorageOptions(uri=dst, storage_id='sqlite3'), ConverterOptions('', ''))
    w.create_topic(TopicMetadata(name=CLOUD, type=types[CLOUD],
                                 serialization_format='cdr'))
    w.create_topic(TopicMetadata(name=OUT_IMU, type='sensor_msgs/msg/Imu',
                                 serialization_format='cdr'))

    imu_cls = get_message(types[IMU])
    r = reader([CLOUD, IMU])
    n_cloud = n_imu = n_skip = 0

    while r.has_next():
        name, d, t_ns = r.read_next()

        if name == CLOUD:
            w.write(CLOUD, d, t_ns)          # 원본 바이트 그대로
            n_cloud += 1
            continue

        # IMU: 자이로는 그대로, 가속도는 본체 것을 LiDAR 프레임으로 회전
        if t_ns < at[0]:
            n_skip += 1                       # 가속도보다 앞선 구간은 버린다
            continue
        i = int(np.searchsorted(at, t_ns, side='right')) - 1
        acc_lidar = R_LB @ (av[i] * ACC_SCALE_BODY)

        m = deserialize_message(d, imu_cls)
        out = Imu()
        out.header.stamp = m.header.stamp     # L1 타임스탬프 유지
        out.header.frame_id = 'utlidar_lidar'
        out.angular_velocity = m.angular_velocity
        out.linear_acceleration.x = float(acc_lidar[0])
        out.linear_acceleration.y = float(acc_lidar[1])
        out.linear_acceleration.z = float(acc_lidar[2])
        out.orientation_covariance[0] = -1.0  # orientation 미사용 표시

        w.write(OUT_IMU, serialize_message(out), t_ns)
        n_imu += 1

    del w
    print(f'    {CLOUD:22s} {n_cloud:,}')
    print(f'    {OUT_IMU:22s} {n_imu:,}   (가속도 이전 {n_skip} 개 건너뜀)')
    print(f'\n확인:\n  ros2 bag info {dst}')
    print(f'\n재생:\n  ros2 bag play {dst} -r 0.5')


if __name__ == '__main__':
    main()
