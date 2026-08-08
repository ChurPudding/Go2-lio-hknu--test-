#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ground_inspect.py — 어디를 바닥으로 잡고 있는지 눈으로 본다

  pcd_to_grid.py 는 전체 점군의 최빈 높이 하나를 지면으로 삼고
  거기에 상대 높이대를 적용한다. 즉 전 영역에 같은 기준선 하나다.

  이 스크립트는 세 가지를 보여준다:
    1) 바닥으로 분류된 점 / 장애물로 분류된 점 / 그 위 점 을 색으로 구분
    2) 격자 칸마다 실제 바닥 높이를 재서, 기준선 하나로 충분한지 확인
    3) 하한을 바꿨을 때 바닥이 얼마나 섞여 들어오는지 수치화

  사용: python3 ground_inspect.py [pcd] [격자해상도] [하한] [상한]
  예:   python3 ground_inspect.py ~/fastlio_ws/results/odommap_v2/scans.pcd 0.10 0.20 1.50
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    pcd = (sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        '~/fastlio_ws/results/odommap_v2/scans.pcd'))
    res = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
    zlo = float(sys.argv[3]) if len(sys.argv) > 3 else 0.20
    zhi = float(sys.argv[4]) if len(sys.argv) > 4 else 1.50
    outdir = os.path.dirname(pcd)

    import open3d as o3d
    P = np.asarray(o3d.io.read_point_cloud(pcd).points)
    print(f'점 {len(P):,} 개')

    # ---- 전역 지면 (pcd_to_grid.py 와 동일 방식) ----
    hist, edges = np.histogram(P[:, 2], bins=200)
    k = int(np.argmax(hist))
    g0 = float((edges[k] + edges[k + 1]) / 2)
    print(f'전역 지면 추정 z = {g0:+.3f} m')
    print(f'장애물 높이대    = {g0+zlo:+.3f} ~ {g0+zhi:+.3f} m\n')

    # ---- 칸별 실제 바닥 높이 ----
    x0, y0 = P[:, 0].min(), P[:, 1].min()
    ix = ((P[:, 0] - x0) / res).astype(int)
    iy = ((P[:, 1] - y0) / res).astype(int)
    W, H = ix.max() + 1, iy.max() + 1
    flat = iy * W + ix

    order = np.lexsort((P[:, 2], flat))       # 칸별로 z 오름차순 정렬
    fs, zs = flat[order], P[order, 2]
    starts = np.searchsorted(fs, np.unique(fs))
    ends = np.append(starts[1:], len(fs))
    cells = np.unique(fs)

    # 칸별 하위 10% 분위수를 그 칸의 바닥으로 본다.
    # 단, 전역 지면 부근에 점이 실제로 있는 칸만 — 벽만 있는 칸의
    # 최하단을 '바닥'으로 잡으면 퍼짐이 허위로 부풀어 오른다.
    NEAR = 0.30
    gcell = np.full(len(cells), np.nan)
    cnt = np.zeros(len(cells), dtype=int)
    for i, (a, b) in enumerate(zip(starts, ends)):
        n = b - a
        cnt[i] = n
        if n < 5:
            continue
        seg = zs[a:b]
        near = seg[np.abs(seg - g0) < NEAR]
        if len(near) >= 3:
            gcell[i] = np.percentile(near, 10)

    ok = ~np.isnan(gcell)
    gv = gcell[ok]
    print(f'격자 {W} x {H},  바닥 점이 있는 칸 {ok.sum():,} / {len(cells):,} '
          f'({100*ok.sum()/max(len(cells),1):.0f} %)\n')
    print('### 칸별 바닥 높이 분포\n')
    print('| 분위 | 값 (m) |')
    print('|---|---|')
    for q in (1, 5, 25, 50, 75, 95, 99):
        print(f'| p{q} | {np.percentile(gv, q):+.3f} |')
    spread = np.percentile(gv, 95) - np.percentile(gv, 5)
    print(f'\n칸별 바닥 높이 퍼짐 (p5~p95) : **{spread:.3f} m**')
    if spread < zlo * 0.5:
        print('→ 전역 기준선 하나로 충분하다.')
    else:
        print(f'→ 퍼짐이 하한 {zlo:.2f} m 의 절반을 넘는다.')
        print('  일부 구역에서 바닥이 장애물로 섞이거나 벽 하단이 잘린다.')

    # ---- 하한별 바닥 혼입량 ----
    print('\n### 하한을 바꿨을 때 바닥이 섞이는 정도\n')
    print('| 하한 | 그 띠의 점 수 | 그 중 칸바닥 +5cm 이내 | 바닥 혼입률 |')
    print('|---|---|---|---|')
    gmap = np.full(W * H, np.nan)
    gmap[cells[ok]] = gv
    pg = gmap[flat]                            # 각 점이 속한 칸의 바닥 높이
    valid = ~np.isnan(pg)
    for lo in (0.10, 0.15, 0.20, 0.25):
        band = valid & (P[:, 2] > g0 + lo) & (P[:, 2] < g0 + zhi)
        near = band & (P[:, 2] < pg + 0.05)
        r = 100 * near.sum() / max(band.sum(), 1)
        mark = '  ← 현행' if abs(lo - zlo) < 1e-9 else ''
        print(f'| {lo:.2f} m | {band.sum():,} | {near.sum():,} | '
              f'**{r:.1f} %**{mark} |')

    # ---- 그림 ----
    sub = np.random.default_rng(0).permutation(len(P))[:600000]
    S = P[sub]
    cls = np.full(len(S), 2)                   # 2 = 그 위
    cls[S[:, 2] <= g0 + zlo] = 0               # 0 = 바닥으로 버려짐
    cls[(S[:, 2] > g0 + zlo) & (S[:, 2] < g0 + zhi)] = 1   # 1 = 장애물

    fig = plt.figure(figsize=(16, 13))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.25, 1, 1])

    ax0 = fig.add_subplot(gs[0, 0])
    ax1 = fig.add_subplot(gs[0, 1])
    ax = [ax0, ax1]

    for c, col, lab in ((0, '#85B7EB', f'ground (z < {zlo:.2f})'),
                        (2, '#D3D1C7', f'above (z > {zhi:.2f})'),
                        (1, '#A32D2D', f'obstacle ({zlo:.2f}~{zhi:.2f})')):
        m = cls == c
        ax[0].scatter(S[m, 0], S[m, 1], s=.2, c=col, label=lab, linewidths=0)
    ax[0].legend(markerscale=40, loc='upper right')
    ax[0].set_title('top view — point classification')

    img = np.full((H, W), np.nan)
    img.ravel()[cells[ok]] = gv
    im = ax[1].imshow(img, origin='lower', cmap='coolwarm',
                      extent=(x0, x0 + W * res, y0, y0 + H * res),
                      vmin=np.percentile(gv, 2), vmax=np.percentile(gv, 98))
    plt.colorbar(im, ax=ax[1], label='cell ground height [m]')
    ax[1].set_title(f'ground height per cell   (global {g0:+.3f} m)')

    for a in ax:
        a.set_aspect('equal'); a.grid(alpha=.25)
        a.set_xlabel('x [m]'); a.set_ylabel('y [m]')

    # ---- 측면 단면 (x-z, y-z) ----
    for row, (axis, other, name) in enumerate(
            ((0, 1, 'x'), (1, 0, 'y')), start=1):
        a = fig.add_subplot(gs[row, :])
        for c, col, lab in ((0, '#85B7EB', 'ground'),
                            (2, '#D3D1C7', 'above'),
                            (1, '#A32D2D', 'obstacle')):
            m = cls == c
            a.scatter(S[m, axis], S[m, 2], s=.15, c=col, label=lab,
                      linewidths=0, alpha=.5)
        a.axhline(g0, color='#0F6E56', lw=1.2, ls='-', label=f'ground {g0:+.2f}')
        a.axhline(g0 + zlo, color='#185FA5', lw=1.2, ls='--',
                  label=f'lower {zlo:.2f}')
        a.axhline(g0 + zhi, color='#BA7517', lw=1.2, ls='--',
                  label=f'upper {zhi:.2f}')
        a.set_xlabel(f'{name} [m]'); a.set_ylabel('z [m]')
        a.set_ylim(g0 - 0.8, g0 + 3.4)
        a.grid(alpha=.25)
        a.set_title(f'side view — {name}-z cross section')
        if row == 1:
            a.legend(markerscale=40, ncol=6, loc='upper right', fontsize=9)

    plt.tight_layout()
    out = os.path.join(outdir, 'ground_inspect.png')
    plt.savefig(out, dpi=115)
    print(f'\n그림: {out}')
    print('\n[위 왼쪽]  파랑=바닥으로 버려진 점, 빨강=장애물, 회색=상한 위')
    print('[위 오른쪽] 칸마다 실제 바닥 높이. 색이 고르면 전역 기준선으로 충분')
    print('[아래 2장] 옆에서 본 단면. 초록 실선=지면, 파란 점선=하한,')
    print('           주황 점선=상한. 하한선이 바닥 띠를 지나면 바닥이 섞인다.')


if __name__ == '__main__':
    main()
