#!/usr/bin/env python3
"""
lidar_pattern_shift.py - 반복 주기 q 프레임 뒤 스캔 패턴이 방위각으로 몇 도 밀렸는가

lidar_pattern_repeat.py 가 q=9 를 확정했으나, 위상 관측량으로 '첫 점의 방위각'을
썼기 때문에 나머지 값이 신뢰 불가였다. 여기서는 점 1500개 전체의 방위각
히스토그램을 원형 상호상관으로 맞춰 패턴 이동량 delta 를 직접 잰다.

  w_prec = delta / (q * T_frame)

예측: 관측 -12.78 deg/s 가 세차라면 delta = -12.78 * 9 * 0.064964 = -7.47 deg

사용:
  python3 lidar_pattern_shift.py ~/data/bags/lio_test_bag_loop_run1 \
          --start 234.5 --dur 34.0 --q 9
"""
import argparse, sys
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

OBS = -12.78
NB = 1440                      # 0.25 deg 칸
SMOOTH = 3.0                   # 히스토그램 평활 sigma [칸]


def circ_smooth(h, sigma):
    n = len(h)
    k = np.arange(n)
    k = np.minimum(k, n - k)
    g = np.exp(-0.5 * (k / sigma) ** 2)
    return np.real(np.fft.ifft(np.fft.fft(h) * np.fft.fft(g / g.sum())))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/utlidar/cloud')
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--dur', type=float, default=30.0)
    ap.add_argument('--q', type=int, default=9)
    a = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))

    t0 = None
    stamps, H = [], []
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
        rec = pc2.read_points(msg, field_names=['x', 'y'], skip_nans=True)
        x = np.asarray(rec['x'], dtype=np.float64)
        y = np.asarray(rec['y'], dtype=np.float64)
        keep = np.hypot(x, y) > 0.5
        if keep.sum() < 200:
            continue
        az = np.arctan2(y[keep], x[keep])
        ib = ((az + np.pi) / (2 * np.pi) * NB).astype(int) % NB
        h = np.bincount(ib, minlength=NB).astype(np.float64)
        h = circ_smooth(h, SMOOTH)
        h -= h.mean()
        h /= (np.linalg.norm(h) + 1e-12)
        stamps.append(rel)
        H.append(h)

    n = len(H)
    if n < a.q + 20:
        print('프레임이 너무 적습니다.'); sys.exit(1)
    T = np.median(np.diff(np.array(stamps)))
    H = np.array(H)
    F = np.fft.fft(H, axis=1)

    print(f'프레임 {n}개, 주기 {T*1e3:.3f} ms, 칸 {360/NB:.3f} deg')
    print(f'{"q":>4} {"delta[deg]":>12} {"corr":>8} {"w_prec[deg/s]":>15}'
          f' {"관측대비":>10}')

    for q in sorted({a.q, 2 * a.q, 1}):
        if q >= n:
            continue
        C = np.real(np.fft.ifft(np.conj(F[:-q]) * F[q:], axis=1)).mean(axis=0)
        i = int(np.argmax(C))
        # 포물선 보간
        ym1, y0, yp1 = C[(i-1) % NB], C[i], C[(i+1) % NB]
        d = 0.5 * (ym1 - yp1) / (ym1 - 2*y0 + yp1 + 1e-15)
        shift = i + d
        if shift > NB / 2:
            shift -= NB
        delta = shift * 360.0 / NB
        w = delta / (q * T)
        print(f'{q:>4} {delta:12.3f} {y0:8.4f} {w:15.3f}'
              f' {abs((w-OBS)/OBS)*100:9.1f}%')

    C = np.real(np.fft.ifft(np.conj(F[:-a.q]) * F[a.q:], axis=1)).mean(axis=0)
    i = int(np.argmax(C))
    ym1, y0, yp1 = C[(i-1) % NB], C[i], C[(i+1) % NB]
    d = 0.5 * (ym1 - yp1) / (ym1 - 2*y0 + yp1 + 1e-15)
    shift = i + d
    if shift > NB / 2:
        shift -= NB
    delta = shift * 360.0 / NB
    w = delta / (a.q * T)
    print(f'\n예측(세차 가설): delta = {OBS * a.q * T:+.3f} deg')
    print(f'실측            : delta = {delta:+.3f} deg  -> w_prec {w:+.3f} deg/s')
    if abs((w - OBS) / OBS) < 0.05:
        print('-> 채택: 정지 드리프트 -12.78 deg/s 는 스캔 패턴 세차다')
    elif abs(delta) < 0.5:
        print('-> 기각: 패턴은 사실상 제자리 반복. 세차는 존재하지 않는다')
    else:
        print('-> 기각: 세차는 있으나 크기가 관측과 다르다')


if __name__ == '__main__':
    main()
