#!/usr/bin/env python3
"""
lidar_pattern_repeat.py - L1 스캔 패턴의 반복 주기 q 와 세차율 측정

lidar_spin_check.py 의 두 결함을 고친 것:
  (1) ring 순서 때문에 방위각이 단조증가하지 않음 -> time 필드로 정렬 후 회귀
  (2) 프레임 위상은 mod 360 이라 그대로 나누면 앨리어싱 -> 반복 주기 q 를 측정해서 나눔

절차:
  1. 프레임마다 (방위각 x 링) 히스토그램 h_k 를 만든다
  2. 지연 L=1..Lmax 에 대해 corr(h_k, h_{k+L}) 을 k 로 평균 -> S(L)
  3. S(L) 이 최대인 L 이 반복 주기 q
  4. w_prec = ((q * dphi) mod 360) / (q * T_frame)

사용:
  source /opt/ros/humble/setup.bash
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
  python3 lidar_pattern_repeat.py ~/data/bags/lio_test_bag_loop_run1 --start 234.5 --dur 34.0
"""
import argparse, sys
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

OBS = -12.78          # 실험 7 관측 정지 드리프트 [deg/s]
NAZ = 360             # 방위각 히스토그램 칸 수 (1 deg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/utlidar/cloud')
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--dur', type=float, default=30.0)
    ap.add_argument('--lmax', type=int, default=60, help='조사할 최대 지연')
    a = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    if a.topic not in {t.name for t in reader.get_all_topics_and_types()}:
        print(f'토픽 없음: {a.topic}'); sys.exit(1)

    t0 = None
    stamps, phase, hists, rot = [], [], [], []
    nring = 1

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
        names = ['x', 'y', 'ring', 'time']
        rec = pc2.read_points(msg, field_names=names, skip_nans=True)
        x = np.asarray(rec['x'], dtype=np.float64)
        y = np.asarray(rec['y'], dtype=np.float64)
        rg = np.asarray(rec['ring'], dtype=np.int32)
        tp = np.asarray(rec['time'], dtype=np.float64)

        keep = np.hypot(x, y) > 0.5
        if keep.sum() < 100:
            continue
        x, y, rg, tp = x[keep], y[keep], rg[keep], tp[keep]

        # 단위 정규화 (s / us / ns 자동)
        span = tp.max() - tp.min()
        if span > 1e6:
            tp = tp / 1e9
        elif span > 1e3:
            tp = tp / 1e6
        tp = tp - tp.min()

        o = np.argsort(tp)                      # 시간순 정렬 -> ring 뒤섞임 해소
        x, y, rg, tp = x[o], y[o], rg[o], tp[o]
        az = np.arctan2(y, x)

        nring = max(nring, int(rg.max()) + 1)
        ib = ((az + np.pi) / (2 * np.pi) * NAZ).astype(int) % NAZ
        h = np.zeros(NAZ * 64, dtype=np.float32)
        np.add.at(h, np.minimum(rg, 63) * NAZ + ib, 1.0)

        stamps.append(rel)
        phase.append(az[0])
        hists.append(h)
        if tp.max() > 1e-4:
            rot.append(np.polyfit(tp, np.unwrap(az), 1)[0])

    n = len(stamps)
    if n < 30:
        print('프레임이 너무 적습니다.'); sys.exit(1)

    stamps = np.array(stamps)
    T = np.median(np.diff(stamps))
    H = np.array(hists)[:, :nring * NAZ]
    H = H - H.mean(axis=1, keepdims=True)
    H /= (np.linalg.norm(H, axis=1, keepdims=True) + 1e-9)

    print(f'프레임 수 {n},  주기 {T*1e3:.3f} ms ({1/T:.4f} Hz),  링 {nring}개')
    if rot:
        w = np.median(rot)
        print(f'로터 각속도 (time 정렬 후) : {np.rad2deg(w):.1f} deg/s'
              f' = {w/(2*np.pi):.3f} Hz')

    ph = np.array(phase)
    dph = np.rad2deg((np.diff(ph) + np.pi) % (2*np.pi) - np.pi)
    dphi = np.median(dph) % 360.0
    print(f'프레임당 위상 진행 dphi   : {dphi:.4f} deg'
          f'  (MAD {np.median(np.abs(dph - np.median(dph))):.4f})')

    print('\n[반복 주기 탐색] S(L) = <corr(h_k, h_k+L)>')
    S = []
    for L in range(1, min(a.lmax, n - 5) + 1):
        S.append((H[:-L] * H[L:]).sum(axis=1).mean())
    S = np.array(S)
    order = np.argsort(S)[::-1]
    print('  상위 6개 지연:')
    for i in order[:6]:
        L = i + 1
        rem = (L * dphi) % 360.0
        if rem > 180:
            rem -= 360.0
        print(f'    L={L:3d}  S={S[i]:.4f}   나머지 {rem:+8.3f} deg'
              f'   -> w_prec {rem/(L*T):+9.3f} deg/s')

    q = int(order[0]) + 1
    rem = (q * dphi) % 360.0
    if rem > 180:
        rem -= 360.0
    w_prec = rem / (q * T)
    print(f'\n[판정] 반복 주기 q = {q},  S = {S[q-1]:.4f}'
          f'  (2위 대비 {S[q-1]-S[order[1]]:+.4f})')
    print(f'  세차율 w_prec = {w_prec:+.3f} deg/s')
    print(f'  관측 {OBS:.2f} deg/s 대비 {abs((w_prec-OBS)/OBS)*100:.1f} %')
    print('  -> 채택: 정지 드리프트는 스캔 패턴 세차와 일치'
          if abs((w_prec - OBS) / OBS) < 0.05 else
          '  -> 기각: 세차는 정지 드리프트의 크기를 설명하지 않는다')


if __name__ == '__main__':
    main()
