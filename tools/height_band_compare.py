#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
height_band_compare.py — 높이 필터 범위를 여러 개로 잘라 한 번에 비교한다

  높이대는 목적에 따라 다르다:
    · A* 통행 판정  → 로봇이 실제로 못 지나가는 높이만
    · 벽 윤곽 추출  → 가구 위, 천장 아래
    · 낮은 장애물   → 문턱·몰드 (바닥 잡음과 섞이기 쉬움)

  지면 높이는 최빈값으로 자동 추정하고, 각 띠는 그 지면 기준 상대 높이다.

  사용: python3 height_band_compare.py [pcd] [해상도]
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import ndimage

BANDS = [
    ('0.20~1.50  현행 (기준)', 0.20, 1.50),
    ('0.20~1.20', 0.20, 1.20),
    ('0.20~1.00  상한 제안', 0.20, 1.00),
    ('0.15~1.00  상하한 동시', 0.15, 1.00),
    ('0.15~1.50  하한만', 0.15, 1.50),
]


def main():
    pcd = (sys.argv[1] if len(sys.argv) > 1
           else os.path.expanduser(
               '~/fastlio_ws/results/odommap_corrected/scans.pcd'))
    res = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
    outdir = os.path.dirname(pcd)

    import open3d as o3d
    pts = np.asarray(o3d.io.read_point_cloud(pcd).points)
    print(f'점 {len(pts):,} 개')

    hist, edges = np.histogram(pts[:, 2], bins=200)
    k = int(np.argmax(hist))
    ground = float((edges[k] + edges[k + 1]) / 2)
    print(f'지면 추정 z = {ground:.3f} m\n')

    x0, x1 = pts[:, 0].min(), pts[:, 0].max()
    y0, y1 = pts[:, 1].min(), pts[:, 1].max()
    W = int(np.ceil((x1 - x0) / res)) + 2
    H = int(np.ceil((y1 - y0) / res)) + 2

    print('| 높이대 | 점 수 | 장애물 칸 | 자유공간 덩어리 | 최대 덩어리 |')
    print('|---|---|---|---|---|')

    grids = []
    for label, lo, hi in BANDS:
        m = (pts[:, 2] >= ground + lo) & (pts[:, 2] <= ground + hi)
        sel = pts[m]
        G = np.zeros((H, W), dtype=bool)
        if len(sel):
            ix = ((sel[:, 0] - x0) / res).astype(int)
            iy = ((sel[:, 1] - y0) / res).astype(int)
            G[iy, ix] = True

        free = ~G
        lab, n = ndimage.label(free)
        sizes = np.bincount(lab.ravel())[1:]
        big = sizes.max() if len(sizes) else 0
        frac = 100 * big / max(free.sum(), 1)
        grids.append((label, G, frac))

        print(f'| {label} | {len(sel):,} | {G.sum():,} ({100*G.mean():.1f}%) '
              f'| {n:,} | **{frac:.1f}%** |')

        np.save(os.path.join(outdir, f'grid_band_{lo:.2f}_{hi:.2f}.npy'), G)

    fig, ax = plt.subplots(len(BANDS), 1, figsize=(15, 4.2 * len(BANDS)))
    for a, (label, G, frac) in zip(np.atleast_1d(ax), grids):
        a.imshow(G, cmap='binary', origin='lower',
                 extent=(x0, x0 + W * res, y0, y0 + H * res),
                 interpolation='nearest')
        a.set_title(f'{label}   —   최대 자유공간 {frac:.1f}%', fontsize=12)
        a.set_xlabel('x [m]'); a.set_ylabel('y [m]')
        a.set_aspect('equal'); a.grid(alpha=.25)
    plt.tight_layout()
    out = os.path.join(outdir, 'height_bands.png')
    plt.savefig(out, dpi=110)
    print(f'\n그림: {out}')

    print('\n### 읽는 법\n')
    print('· 최대 자유공간 비율이 높다  = 층이 하나로 이어짐 (A* 에 유리)')
    print('· 장애물 칸이 지나치게 적다  = 벽이 끊겨 A* 가 벽을 통과할 수 있음')
    print('· 둘의 균형을 그림으로 보고 고를 것. 숫자만으로는 못 정한다.')


if __name__ == '__main__':
    main()
