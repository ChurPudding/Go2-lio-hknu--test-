#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pillar_inspect.py — 특정 기둥을 확대해 뭉개진 정도를 잰다

  기둥은 단면이 일정한 수직 구조라, 지도에서 뭉개진 정도가 곧 정합 오차다.
  실측(줄자)과 비교하면 축척 검증에도 쓸 수 있다.

  보는 것:
    1) 위에서 본 단면 — 여러 겹으로 갈라졌는지
    2) 높이별 단면    — 위아래에서 중심이 어긋나는지 (기울어짐)
    3) 측면 x-z, y-z  — 수직으로 곧게 서 있는지
    4) 굵기 추정      — 실측과 비교할 숫자

  사용: python3 pillar_inspect.py <pcd> <cx> <cy> [반경] [zlo] [zhi]
  예:   python3 pillar_inspect.py ~/fastlio_ws/results/odommap_v2/scans.pcd 12 -5 2.5
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    pcd = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        '~/fastlio_ws/results/odommap_v2/scans.pcd')
    cx = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
    cy = float(sys.argv[3]) if len(sys.argv) > 3 else -5.0
    R = float(sys.argv[4]) if len(sys.argv) > 4 else 2.5
    zlo = float(sys.argv[5]) if len(sys.argv) > 5 else 0.20
    zhi = float(sys.argv[6]) if len(sys.argv) > 6 else 1.50
    outdir = os.path.dirname(pcd)

    import open3d as o3d
    P = np.asarray(o3d.io.read_point_cloud(pcd).points)

    hist, edges = np.histogram(P[:, 2], bins=200)
    k = int(np.argmax(hist))
    g0 = float((edges[k] + edges[k + 1]) / 2)

    m = (np.abs(P[:, 0] - cx) < R) & (np.abs(P[:, 1] - cy) < R) \
        & (P[:, 2] > g0 + zlo) & (P[:, 2] < g0 + zhi)
    Q = P[m]
    print(f'중심 ({cx}, {cy}), 반경 {R} m, 높이 {g0+zlo:+.2f}~{g0+zhi:+.2f} m')
    print(f'해당 점 {len(Q):,} 개\n')
    if len(Q) < 200:
        print('점이 너무 적습니다. 중심 좌표나 반경을 확인하세요.')
        return

    # ---- 높이별 중심 이동 (기울어짐 / 어긋남) ----
    print('### 높이별 단면\n')
    print('| 높이 구간 | 점 수 | 중심 x | 중심 y | x 폭 p5~p95 | y 폭 p5~p95 |')
    print('|---|---|---|---|---|---|')
    levels = np.linspace(g0 + zlo, g0 + zhi, 7)
    cents = []
    for i in range(len(levels) - 1):
        s = Q[(Q[:, 2] >= levels[i]) & (Q[:, 2] < levels[i + 1])]
        if len(s) < 30:
            print(f'| {levels[i]:+.2f}~{levels[i+1]:+.2f} | {len(s)} | — | — | — | — |')
            continue
        mx, my = np.median(s[:, 0]), np.median(s[:, 1])
        wx = np.percentile(s[:, 0], 95) - np.percentile(s[:, 0], 5)
        wy = np.percentile(s[:, 1], 95) - np.percentile(s[:, 1], 5)
        cents.append((levels[i], mx, my))
        print(f'| {levels[i]:+.2f}~{levels[i+1]:+.2f} | {len(s):,} | {mx:.3f} | '
              f'{my:.3f} | {wx:.3f} | {wy:.3f} |')

    if len(cents) >= 2:
        C = np.array(cents)
        drift = float(np.hypot(C[-1, 1] - C[0, 1], C[-1, 2] - C[0, 2]))
        span = C[-1, 0] - C[0, 0]
        print(f'\n바닥→위쪽 중심 이동 : **{drift:.3f} m** (높이차 {span:.2f} m)')
        if drift > 0.15:
            print('→ 기둥이 기울어져 보인다. 자세 추정의 pitch/roll 잔차 또는 겹침.')
        else:
            print('→ 수직으로 곧게 서 있다.')

    # ---- 굵기 추정 ----
    wx = np.percentile(Q[:, 0], 95) - np.percentile(Q[:, 0], 5)
    wy = np.percentile(Q[:, 1], 95) - np.percentile(Q[:, 1], 5)
    print(f'\n### 굵기 (전체 높이 통합)\n')
    print(f'x 방향 p5~p95 : **{wx:.3f} m**')
    print(f'y 방향 p5~p95 : **{wy:.3f} m**')
    print('\n실제 기둥을 줄자로 재서 비교하면 축척 검증이 된다.')
    print('지도가 더 두껍게 나오면 그 차이가 곧 뭉개진 양이다.')

    # ---- 그림 ----
    fig, ax = plt.subplots(2, 2, figsize=(13, 12))

    a = ax[0, 0]
    a.scatter(Q[:, 0], Q[:, 1], s=1.5, c=Q[:, 2], cmap='viridis', linewidths=0)
    a.set_title('top view (color = height)')
    a.set_xlabel('x [m]'); a.set_ylabel('y [m]'); a.set_aspect('equal')

    a = ax[0, 1]
    cols = plt.cm.plasma(np.linspace(0, .9, len(levels) - 1))
    for i in range(len(levels) - 1):
        s = Q[(Q[:, 2] >= levels[i]) & (Q[:, 2] < levels[i + 1])]
        if len(s) < 30:
            continue
        a.scatter(s[:, 0], s[:, 1], s=2, color=cols[i], linewidths=0,
                  label=f'{levels[i]:+.2f} m')
    a.legend(fontsize=8, markerscale=4)
    a.set_title('slices by height')
    a.set_xlabel('x [m]'); a.set_ylabel('y [m]'); a.set_aspect('equal')

    for j, (axis, name) in enumerate(((0, 'x'), (1, 'y'))):
        a = ax[1, j]
        a.scatter(Q[:, axis], Q[:, 2], s=1.5, c='#A32D2D', linewidths=0, alpha=.5)
        a.set_xlabel(f'{name} [m]'); a.set_ylabel('z [m]')
        a.set_title(f'side view — {name}-z')
        a.grid(alpha=.25)

    for row in ax:
        for a in row:
            a.grid(alpha=.25)
    plt.tight_layout()
    out = os.path.join(outdir, f'pillar_{cx:g}_{cy:g}.png')
    plt.savefig(out, dpi=120)
    print(f'\n그림: {out}')


if __name__ == '__main__':
    main()
