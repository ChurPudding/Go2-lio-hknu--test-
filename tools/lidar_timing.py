#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_timing.py — L1 라이다의 시간 특성을 bag 에서 직접 잰다

  속도로 인한 번짐을 계산하려면 세 가지가 필요하다:
    1. 스캔 하나를 모으는 데 걸리는 시간  (= 그 동안 로봇이 이동한 거리)
    2. 초당 점 개수                        (= 밀도)
    3. 스캔 간격                           (= 발행 주기)

  L1 은 비반복 스캔(non-repetitive) 방식이라 회전식 라이다와 달리
  '1회전' 개념이 명확하지 않다. 대신 한 프레임에 담긴 점들의
  타임스탬프 범위가 실질적인 '적분 시간'이 된다.

  point 필드에 시간 정보(t / time / timestamp / offset_time)가 있으면
  프레임 내부 적분 시간을 직접 잰다. 없으면 프레임 간격으로 추정한다.

  사용: python3 lidar_timing.py <bag> [토픽]
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
    bag = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        '~/data/bags/indoor/floor_0805_1720')
    topic = sys.argv[2] if len(sys.argv) > 2 else '/utlidar/cloud'

    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=detect_storage(bag)),
           ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if topic not in types:
        print(f'{topic} 없음. 점군 토픽 목록:')
        for n, t in types.items():
            if 'PointCloud2' in t:
                print(f'  {n}')
        return
    cls = get_message(types[topic])

    print(f'bag   : {bag}')
    print(f'topic : {topic}\n')

    stamps, npts = [], []
    fields = None
    tfield = None
    inner = []          # 프레임 내부 적분 시간
    ODOM = '/utlidar/robot_odom'
    ot, oxy = [], []
    k = 0

    while r.has_next():
        n, d, _ = r.read_next()
        if n == ODOM:
            m = deserialize_message(d, get_message(types[ODOM]))
            p = m.pose.pose.position
            ot.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            oxy.append((p.x, p.y))
            continue
        if n != topic:
            continue
        m = deserialize_message(d, cls)
        if fields is None:
            fields = [(f.name, f.offset, f.datatype, f.count) for f in m.fields]
            print('### 점 필드 구성\n')
            print('| 이름 | offset | datatype | count |')
            print('|---|---|---|---|')
            for nm, off, dt, cnt in fields:
                print(f'| {nm} | {off} | {dt} | {cnt} |')
            names = [f[0] for f in fields]
            for cand in ('t', 'time', 'timestamp', 'offset_time',
                         'time_offset', 'curvature'):
                if cand in names:
                    tfield = cand
                    break
            print(f'\npoint_step={m.point_step}, width={m.width}, '
                  f'height={m.height}, is_dense={m.is_dense}')
            print(f'시간 필드: {tfield if tfield else "없음 (프레임 간격으로 추정)"}\n')

        stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        npts.append(m.width * m.height)

        if tfield and len(inner) < 400:
            try:
                tv = point_cloud2.read_points_numpy(m, field_names=(tfield,),
                                                    skip_nans=True).ravel()
                tv = tv[np.isfinite(tv)]
                if len(tv) > 10:
                    inner.append(float(tv.max() - tv.min()))
            except Exception:
                pass
        k += 1

    stamps = np.array(stamps)
    npts = np.array(npts)
    dt = np.diff(stamps)
    dt = dt[(dt > 0) & (dt < 1.0)]

    print('### 스캔 발행 주기\n')
    print('| 항목 | 값 |')
    print('|---|---|')
    print(f'| 프레임 수 | {len(stamps):,} |')
    print(f'| 구간 길이 | {stamps[-1]-stamps[0]:.1f} s |')
    print(f'| 평균 주기 | {dt.mean()*1000:.2f} ms |')
    print(f'| 발행 주파수 | **{1/dt.mean():.2f} Hz** |')
    print(f'| 주기 표준편차 | {dt.std()*1000:.2f} ms |')
    print(f'| 주기 p1~p99 | {np.percentile(dt,1)*1000:.1f} ~ '
          f'{np.percentile(dt,99)*1000:.1f} ms |')

    print('\n### 점 개수\n')
    print('| 항목 | 값 |')
    print('|---|---|')
    print(f'| 프레임당 평균 | **{npts.mean():,.0f} 점** |')
    print(f'| 프레임당 p1~p99 | {np.percentile(npts,1):,.0f} ~ '
          f'{np.percentile(npts,99):,.0f} |')
    print(f'| 초당 점 개수 | **{npts.mean()/dt.mean():,.0f} 점/s** |')
    print(f'| 전체 점 합계 | {npts.sum():,} |')

    if inner:
        inner = np.array(inner)
        integ = float(np.median(inner))
        print('\n### 프레임 내부 적분 시간 (실측)\n')
        print(f'| 중앙값 | **{integ*1000:.2f} ms** |')
        print(f'| p1~p99 | {np.percentile(inner,1)*1000:.2f} ~ '
              f'{np.percentile(inner,99)*1000:.2f} ms |')
        print(f'\n한 프레임의 점들이 이 시간에 걸쳐 수집된다.')
    else:
        integ = float(dt.mean())
        print('\n### 프레임 내부 적분 시간 (추정)\n')
        print(f'시간 필드가 없어 발행 주기로 대신한다: **{integ*1000:.2f} ms**')
        print('실제 적분 시간은 이보다 짧을 수 있다.')

    # ---- 속도 → 번짐 ----
    if ot:
        ot = np.array(ot); oxy = np.array(oxy)
        v = np.linalg.norm(np.diff(oxy, axis=0), axis=1) / np.maximum(
            np.diff(ot), 1e-6)
        v = v[np.isfinite(v) & (v < 3.0)]
        print('\n### 이동 속도와 그로 인한 번짐\n')
        print('| 속도 | 한 프레임 동안 이동 거리 |')
        print('|---|---|')
        for lab, val in (('평균', np.mean(v)),
                         ('p50', np.percentile(v, 50)),
                         ('p90', np.percentile(v, 90)),
                         ('p99', np.percentile(v, 99)),
                         ('최대', v.max())):
            print(f'| {lab} {val:.2f} m/s | **{val*integ*100:.1f} cm** |')

        print('\n이 값이 디스큐로 보정되지 않고 남는 최대 번짐이다.')
        print('실측된 뭉개짐이 이보다 훨씬 크면, 속도는 주원인이 아니다.')

    print('\n### L1 스캔 방식에 대해\n')
    print('L1 은 비반복 스캔(non-repetitive)이라 회전식처럼 "1회전" 개념이')
    print('뚜렷하지 않다. 시야를 시간에 따라 서로 다른 궤적으로 훑으므로,')
    print('오래 볼수록 조밀해진다. 따라서 "회전 속도" 대신 위의')
    print('발행 주기와 적분 시간이 실질적인 시간 특성이다.')


if __name__ == '__main__':
    main()
