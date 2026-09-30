#!/usr/bin/env python3
"""
lidar_spin_check.py - Unitree L1 로터 회전율 / 스캔 패턴 세차율 측정

가설: 정지 중 관측된 yaw 드리프트(-12.78 deg/s)가
      로터 주기와 프레임 주기의 비정수비에서 오는 스캔 패턴 세차와 같다.

예측: T_frame = 0.1 s, w_prec = -12.78 deg/s  ->  f_rot = 10n - 0.0355 Hz

사용:
  source /opt/ros/humble/setup.bash
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
  python3 lidar_spin_check.py ~/data/bags/lio_test_bag_loop_run1 \
          --topic /utlidar/cloud --start 234.5 --dur 34.0
"""
import argparse, sys
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def open_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/utlidar/cloud')
    ap.add_argument('--start', type=float, default=0.0, help='bag 시작 오프셋 [s]')
    ap.add_argument('--dur', type=float, default=30.0, help='분석 길이 [s]')
    a = ap.parse_args()

    reader = open_bag(a.bag)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if a.topic not in types:
        print(f'토픽 없음: {a.topic}\n사용 가능: {list(types)}'); sys.exit(1)

    t0 = None
    stamps, phase0, phase1, npts, rot_rate = [], [], [], [], []
    tf = None
    shown = False

    while reader.has_next():
        topic, data, tns = reader.read_next()
        if topic != a.topic:
            continue
        t = tns * 1e-9
        if t0 is None:
            t0 = t
        rel = t - t0
        if rel < a.start:
            continue
        if rel > a.start + a.dur:
            break

        msg = deserialize_message(data, PointCloud2)
        fields = [f.name for f in msg.fields]
        if not shown:
            DT = {1: 'int8', 2: 'uint8', 3: 'int16', 4: 'uint16',
                  5: 'int32', 6: 'uint32', 7: 'float32', 8: 'float64'}
            print('[필드] ' + ', '.join(
                f'{f.name}:{DT.get(f.datatype, f.datatype)}@{f.offset}'
                for f in msg.fields))
            tf = next((f for f in ('offset_time', 'time', 't', 'timestamp',
                                   'time_stamp', 'curvature', 'intensity')
                       if f in fields), None)
            print(f'[시간 필드] {tf}')
            shown = True

        names = ['x', 'y', 'z'] + ([tf] if tf else [])
        rec = pc2.read_points(msg, field_names=names, skip_nans=True)
        if rec.shape[0] < 100:
            continue

        x = np.asarray(rec['x'], dtype=np.float64)
        y = np.asarray(rec['y'], dtype=np.float64)
        rho = np.hypot(x, y)
        keep = rho > 0.5
        if keep.sum() < 100:
            continue
        x, y = x[keep], y[keep]
        az = np.unwrap(np.arctan2(y, x))

        stamps.append(rel)
        npts.append(len(x))
        phase0.append(np.arctan2(y[0], x[0]))
        phase1.append(np.arctan2(y[-1], x[-1]))

        if tf:
            tp = np.asarray(rec[tf], dtype=np.float64)[keep]
            tp = tp - tp.min()
            if tp.max() > 1e6:          # ns -> s
                tp = tp / 1e9
            elif tp.max() > 1e3:        # us -> s
                tp = tp / 1e6
            if tp.max() > 1e-4:
                rot_rate.append(np.polyfit(tp, az, 1)[0])

    if len(stamps) < 20:
        print('프레임이 너무 적습니다.'); sys.exit(1)

    stamps = np.array(stamps)
    dT = np.diff(stamps)
    T_frame = np.median(dT)
    print(f'프레임 수      : {len(stamps)}')
    print(f'프레임 주기    : {T_frame*1e3:.3f} ms  (sigma {dT.std()*1e3:.3f} ms)'
          f'  -> {1/T_frame:.4f} Hz')
    print(f'프레임당 점 수 : {np.mean(npts):.0f}')

    if rot_rate:
        w = np.median(rot_rate)
        print(f'\n[로터 각속도] 프레임 내 az vs t 회귀')
        print(f'  {np.rad2deg(w):.1f} deg/s  = {w/(2*np.pi):.4f} Hz'
              f'  (중앙값, N={len(rot_rate)})')
    else:
        print('\n[로터 각속도] 점별 타임스탬프 필드 없음 -> 생략')

    ph = np.unwrap(np.array(phase0))
    k = np.arange(len(ph))
    s_frame = np.polyfit(k, ph, 1)[0]                 # rad / frame
    res = ph - np.polyval(np.polyfit(k, ph, 1), k)
    w_prec = s_frame / T_frame
    print(f'\n[스캔 패턴 세차] 프레임 시작 위상 언랩 회귀')
    print(f'  프레임당 {np.rad2deg(s_frame):+.4f} deg')
    print(f'  세차율   {np.rad2deg(w_prec):+.3f} deg/s   (잔차 RMS '
          f'{np.rad2deg(res.std()):.2f} deg)')

    OBS = -12.78
    print(f'\n[가설 판정]  실험 7 관측 정지 드리프트 = {OBS:.2f} deg/s')
    d = np.rad2deg(w_prec) - OBS
    print(f'  세차율 - 관측 = {d:+.3f} deg/s  ({abs(d/OBS)*100:.1f} %)')
    print('  -> 가설 채택 (5 % 이내)' if abs(d/OBS) < 0.05 else
          '  -> 가설 기각. 세차가 원인이 아니다.')
    f_pred = (2*np.pi - np.deg2rad(OBS)*T_frame) / (2*np.pi*T_frame)
    print(f'\n  참고: w_prec = {OBS} deg/s 를 만드는 로터 주파수 (n=1)'
          f' = {f_pred:.4f} Hz')


if __name__ == '__main__':
    main()
