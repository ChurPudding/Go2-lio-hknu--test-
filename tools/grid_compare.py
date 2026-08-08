#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
grid_compare.py — 무보정 / 보정 격자를 나란히 + 겹쳐서 본다

  두 격자는 원점과 크기가 다르므로 실제 좌표(m)로 맞춰서 그린다.
  겹침 그림에서:
    빨강 = 무보정에만 있는 장애물
    파랑 = 보정에만 있는 장애물
    검정 = 둘 다 장애물 (변하지 않은 벽)

  빨강이 넓게 번져 있고 파랑이 가늘면, 보정으로 벽이 모인 것이다.

  사용: python3 grid_compare.py
"""

import os

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

BASE = os.path.expanduser('~/fastlio_ws/results')
SETS = [('무보정 (odommap)', 'odommap'),
        ('보정 (odommap_corrected)', 'odommap_corrected')]

# 확대해서 볼 구간
ZOOMS = [('corridor  x=20~52', (20, 52), (-12, 0)),
         ('left area  x=-5~22', (-5, 22), (-18, 6))]


def load(path):
    G = np.load(os.path.join(BASE, path, 'grid.npy'))
    with open(os.path.join(BASE, path, 'grid.yaml')) as f:
        meta = yaml.safe_load(f)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h, w = G.shape
    # extent: 실제 좌표 (m)
    ext = (ox, ox + w * res, oy, oy + h * res)
    return (G > 0), ext, res, (ox, oy)


def to_common(mask, ext, res, grid_x, grid_y):
    """실제 좌표 격자에 최근접으로 다시 샘플링."""
    h, w = mask.shape
    ix = ((grid_x - ext[0]) / res).astype(int)
    iy = ((grid_y - ext[2]) / res).astype(int)
    ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    out = np.zeros(grid_x.shape, dtype=bool)
    out[ok] = mask[iy[ok], ix[ok]]
    return out


def main():
    data = []
    for label, path in SETS:
        m, ext, res, org = load(path)
        data.append((label, m, ext, res))
        print(f'{label}: {m.shape}, extent x {ext[0]:.2f}~{ext[1]:.2f}, '
              f'y {ext[2]:.2f}~{ext[3]:.2f}, 장애물 {m.sum():,}')

    # ---------- 1) 나란히 ----------
    fig, ax = plt.subplots(2, 1, figsize=(16, 11))
    for a, (label, m, ext, res) in zip(ax, data):
        a.imshow(m, cmap='binary', origin='lower', extent=ext,
                 interpolation='nearest')
        a.set_title(label, fontsize=13)
        a.set_xlabel('x [m]'); a.set_ylabel('y [m]')
        a.set_aspect('equal'); a.grid(alpha=.25)
        for zl, zx, zy in ZOOMS:
            a.add_patch(Rectangle((zx[0], zy[0]), zx[1]-zx[0], zy[1]-zy[0],
                                  fill=False, ec='tab:orange', lw=1.4, ls='--'))
    plt.tight_layout()
    p1 = os.path.join(BASE, 'compare_sidebyside.png')
    plt.savefig(p1, dpi=115); plt.close()

    # ---------- 공통 좌표계 ----------
    x0 = min(d[2][0] for d in data); x1 = max(d[2][1] for d in data)
    y0 = min(d[2][2] for d in data); y1 = max(d[2][3] for d in data)
    res = data[0][3]
    xs = np.arange(x0, x1, res)
    ys = np.arange(y0, y1, res)
    GX, GY = np.meshgrid(xs, ys)
    MA = to_common(data[0][1], data[0][2], data[0][3], GX, GY)
    MB = to_common(data[1][1], data[1][2], data[1][3], GX, GY)

    rgb = np.ones(GX.shape + (3,))
    both = MA & MB
    onlyA = MA & ~MB
    onlyB = MB & ~MA
    rgb[onlyA] = [0.85, 0.15, 0.15]     # 무보정에만
    rgb[onlyB] = [0.15, 0.35, 0.90]     # 보정에만
    rgb[both] = [0.05, 0.05, 0.05]      # 공통

    print(f'\n공통 장애물 {both.sum():,},  무보정만 {onlyA.sum():,},  '
          f'보정만 {onlyB.sum():,}')

    # ---------- 2) 겹침 (전체 + 확대) ----------
    fig = plt.figure(figsize=(16, 13))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.5, 1, 1])

    a0 = fig.add_subplot(gs[0, :])
    a0.imshow(rgb, origin='lower', extent=(x0, x1, y0, y1),
              interpolation='nearest')
    a0.set_title('overlay  —  red: uncorrected only   blue: corrected only   '
                 'black: both', fontsize=13)
    a0.set_xlabel('x [m]'); a0.set_ylabel('y [m]')
    a0.set_aspect('equal'); a0.grid(alpha=.25)

    for i, (zl, zx, zy) in enumerate(ZOOMS):
        a = fig.add_subplot(gs[1 + i, :])
        a.imshow(rgb, origin='lower', extent=(x0, x1, y0, y1),
                 interpolation='nearest')
        a.set_xlim(*zx); a.set_ylim(*zy)
        a.set_title(f'zoom — {zl}', fontsize=12)
        a.set_xlabel('x [m]'); a.set_ylabel('y [m]')
        a.set_aspect('equal'); a.grid(alpha=.25)

    plt.tight_layout()
    p2 = os.path.join(BASE, 'compare_overlay.png')
    plt.savefig(p2, dpi=115); plt.close()

    print(f'\n저장:\n  {p1}\n  {p2}')


if __name__ == '__main__':
    main()
