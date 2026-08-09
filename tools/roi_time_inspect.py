#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
roi_time_inspect.py — 특정 구역을 '언제 봤는지'로 색칠해 본다

  전역 겹침 지표(전반-후반 ICP)는 지도 전체를 강체로 한 번 맞춘 값이라,
  국소적으로 몇 m 어긋나 있어도 방향이 제각각이면 상쇄되어 작게 나온다.
  그래서 국소 불일치는 따로 봐야 한다.

  이 스크립트는 관심 구역의 점만 뽑아 '관측 시각'으로 색칠한다.
    · 시간대별로 덩어리가 갈린다  → 같은 곳을 여러 번 봤는데 위치가 어긋남
                                    (루프 클로저 제약이 더 필요)
    · 시간 색이 고르게 섞여 있다  → 한 번의 통과 안에서 흐려진 것
                                    (디스큐/센서 문제 또는 구조물 해석 오류)

  loop_correct_v2 와 같은 증분 보정을 적용한 뒤 뽑는다.

  사용:
    python3 roi_time_inspect.py <bag> --xmin 9 --xmax 15 --ymin -8 --ymax -3
    python3 roi_time_inspect.py <bag> --xmin 9 --xmax 15 --ymin -8 --ymax -3 --raw
"""

import argparse
import math
import os

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


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


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rot(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def open_reader(bag):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=detect_storage(bag)),
           ConverterOptions('', ''))
    return r


def deform(P, TH, s, target_p, target_th):
    n = len(P)
    stot = s[-1] if s[-1] > 0 else 1.0
    dp = np.zeros((n, 2)); dth = np.zeros(n)
    for k in range(1, n):
        dp[k] = rot(TH[k - 1]).T @ (P[k] - P[k - 1])
        dth[k] = wrap(TH[k] - TH[k - 1])
    dyaw_tot = wrap(target_th - TH[-1])
    ds = np.diff(s, prepend=s[0])
    dth2 = dth + dyaw_tot * (ds / stot)
    TH2 = np.zeros(n); P2 = np.zeros((n, 2))
    TH2[0], P2[0] = TH[0], P[0]
    for k in range(1, n):
        TH2[k] = TH2[k - 1] + dth2[k]
        P2[k] = P2[k - 1] + rot(TH2[k - 1]) @ dp[k]
    P2 = P2 + (s / stot)[:, None] * (target_p - P2[-1])
    return P2, TH2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', nargs='?',
                    default=os.path.expanduser('~/data/bags/indoor/floor_0805_1720'))
    ap.add_argument('--xmin', type=float, default=9.0)
    ap.add_argument('--xmax', type=float, default=15.0)
    ap.add_argument('--ymin', type=float, default=-8.0)
    ap.add_argument('--ymax', type=float, default=-3.0)
    ap.add_argument('--zlo', type=float, default=0.20)
    ap.add_argument('--zhi', type=float, default=1.50)
    ap.add_argument('--end-dx', type=float, default=0.0)
    ap.add_argument('--end-dy', type=float, default=0.0)
    ap.add_argument('--end-dyaw', type=float, default=0.0)
    ap.add_argument('--raw', action='store_true', help='보정 없이 원본 자세로')
    ap.add_argument('--gap', type=float, default=20.0,
                    help='이 초 이상 비면 다른 방문으로 본다')
    ap.add_argument('--out', default=os.path.expanduser(
        '~/fastlio_ws/results/odommap_v2'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    CLOUD = '/utlidar/cloud_deskewed'
    ODOM = '/utlidar/robot_odom'

    r = open_reader(a.bag)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    odom_cls = get_message(types[ODOM])
    cloud_cls = get_message(types[CLOUD])

    print('[1] 오도메트리 읽는 중...')
    T, P, TH = [], [], []
    while r.has_next():
        n, d, _ = r.read_next()
        if n != ODOM:
            continue
        m = deserialize_message(d, odom_cls)
        p = m.pose.pose.position
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        P.append((p.x, p.y)); TH.append(yaw_of(m.pose.pose.orientation))
    T = np.array(T); P = np.array(P); TH = np.array(TH)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])

    if a.raw:
        P2, TH2 = P.copy(), TH.copy()
        print('    --raw : 보정 없이 원본 자세 사용')
    else:
        tth = TH[0] + math.radians(a.end_dyaw)
        tp = P[0] + rot(TH[0]) @ np.array([a.end_dx, a.end_dy])
        P2, TH2 = deform(P, TH, s, tp, tth)
        print('    증분 보정 적용')

    print('\n[2] 관심 구역 점 수집...')
    r = open_reader(a.bag)
    X, Y, Z, TS = [], [], [], []
    k = 0
    while r.has_next():
        n, d, _ = r.read_next()
        if n != CLOUD:
            continue
        m = deserialize_message(d, cloud_cls)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        i = int(np.searchsorted(T, st)); i = max(0, min(i, len(T) - 1))
        R = rot(TH2[i] - TH[i]); t = P2[i] - R @ P[i]

        arr = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'),
                                             skip_nans=True)
        arr = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
        arr = arr[np.isfinite(arr).all(axis=1)]
        if len(arr):
            xy = (R @ arr[:, :2].T).T + t
            msk = ((xy[:, 0] > a.xmin) & (xy[:, 0] < a.xmax) &
                   (xy[:, 1] > a.ymin) & (xy[:, 1] < a.ymax))
            if msk.any():
                X.append(xy[msk, 0]); Y.append(xy[msk, 1])
                Z.append(arr[msk, 2]); TS.append(np.full(msk.sum(), st))
        k += 1
        if k % 1000 == 0:
            print(f'    {k} 프레임')

    if not X:
        print('해당 구역에 점이 없습니다. 범위를 확인하세요.')
        return
    X = np.concatenate(X); Y = np.concatenate(Y)
    Z = np.concatenate(Z); TS = np.concatenate(TS)
    t0 = T[0]
    el = TS - t0

    hist, edges = np.histogram(Z, bins=100)
    g0 = float((edges[int(np.argmax(hist))] + edges[int(np.argmax(hist)) + 1]) / 2)
    hm = (Z > g0 + a.zlo) & (Z < g0 + a.zhi)
    X, Y, el = X[hm], Y[hm], el[hm]
    print(f'\n구역 내 장애물 높이대 점 {len(X):,} 개')

    # ---- 방문 구간 나누기 ----
    u = np.unique(el)
    brk = np.where(np.diff(u) > a.gap)[0]
    bounds = np.concatenate([[u[0]], u[brk + 1], [u[-1] + 1e-6]])
    starts = np.concatenate([[u[0]], u[brk + 1]])
    ends = np.concatenate([u[brk], [u[-1]]])

    print(f'\n### 이 구역을 방문한 시간대 ({a.gap:.0f}초 이상 공백 기준)\n')
    print('| # | 시간 (s) | 점 수 | 중심 x | 중심 y |')
    print('|---|---|---|---|---|')
    cent = []
    for i, (s0, s1) in enumerate(zip(starts, ends), 1):
        m = (el >= s0) & (el <= s1)
        if m.sum() < 200:
            continue
        cx, cy = np.median(X[m]), np.median(Y[m])
        cent.append((i, s0, s1, cx, cy, m.sum()))
        print(f'| {i} | {s0:.0f} ~ {s1:.0f} | {m.sum():,} | {cx:.2f} | {cy:.2f} |')

    if len(cent) >= 2:
        print('\n### 방문 간 중심 어긋남\n')
        print('| 비교 | 거리 (m) |')
        print('|---|---|')
        worst = 0.0
        for i in range(len(cent)):
            for j in range(i + 1, len(cent)):
                d = math.hypot(cent[i][3] - cent[j][3], cent[i][4] - cent[j][4])
                worst = max(worst, d)
                print(f'| {cent[i][0]} vs {cent[j][0]} | **{d:.2f}** |')
        print(f'\n최대 어긋남 **{worst:.2f} m**')
        if worst > 1.0:
            print('→ 같은 곳을 여러 번 봤는데 위치가 어긋났다.')
            print('  국소 드리프트. 중간 루프 제약이 필요하다.')
        else:
            print('→ 방문 간 위치는 일치한다. 뭉개짐의 원인은 다른 데 있다.')
    else:
        print('\n방문 구간이 하나뿐이다.')
        print('→ 한 번의 통과 안에서 흐려진 것. 루프 클로저로는 해결되지 않는다.')

    # ---- 그림 ----
    fig, ax = plt.subplots(1, 2, figsize=(15, 7))
    sc = ax[0].scatter(X, Y, s=1.2, c=el, cmap='turbo', linewidths=0)
    plt.colorbar(sc, ax=ax[0], label='elapsed [s]')
    ax[0].set_title('colored by observation time')

    cols = plt.cm.tab10(np.linspace(0, 1, 10))
    for n_, (i, s0, s1, cx, cy, cnt) in enumerate(cent):
        m = (el >= s0) & (el <= s1)
        ax[1].scatter(X[m], Y[m], s=1.2, color=cols[n_ % 10], linewidths=0,
                      label=f'#{i}  {s0:.0f}~{s1:.0f}s')
        ax[1].plot(cx, cy, 'k+', ms=14, mew=2)
    ax[1].legend(fontsize=8, markerscale=6)
    ax[1].set_title('separated by visit  (+ = centroid)')

    for x in ax:
        x.set_aspect('equal'); x.grid(alpha=.25)
        x.set_xlabel('x [m]'); x.set_ylabel('y [m]')
    plt.tight_layout()
    tag = 'raw' if a.raw else 'corr'
    out = os.path.join(a.out, f'roi_time_{tag}.png')
    plt.savefig(out, dpi=120)
    print(f'\n그림: {out}')


if __name__ == '__main__':
    main()
