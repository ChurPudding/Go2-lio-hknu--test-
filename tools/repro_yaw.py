#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
repro_yaw.py — 실행 간 각도 차이가 '언제' 생겼는지 찾는다

  자세(쿼터니언)를 저장하지 않았으므로 위치에서 진행 방향을 유도한다.
  프레임 k 의 헤딩 = (k+W) 위치 - (k-W) 위치 의 방향.

  판독:
    · 각도 차가 처음부터 일정      → 초기 자세(yaw) 차이. 원리상 yaw 는
                                     자력계 없이 관측 불가하므로 초기화 문제
    · 특정 구간에서 계단처럼 증가  → 그 구간에 원인이 있음 (급회전/기하 퇴화)
    · 완만히 계속 증가             → 자이로 적분 드리프트 누적

  사용: python3 repro_yaw.py [repro 루트] [접두어]
"""

import os
import sys
import numpy as np

W = 15            # 헤딩 계산 반폭 (프레임). 15 ≈ 1초분
MIN_DISP = 0.05   # 이 이하로 움직인 구간은 방향이 무의미하므로 제외 (m)


def load(path):
    d = np.loadtxt(path, delimiter=',', skiprows=1)
    return d[:, 0], d[:, 1:3]     # stamp, xy


def heading(xy):
    """각 프레임의 진행 방향(rad)과 유효 여부."""
    n = len(xy)
    ang = np.full(n, np.nan)
    for k in range(W, n - W):
        d = xy[k + W] - xy[k - W]
        if np.linalg.norm(d) >= MIN_DISP:
            ang[k] = np.arctan2(d[1], d[0])
    return ang


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fastlio_ws/results/repro')
    prefix = sys.argv[2] if len(sys.argv) > 2 else 'base_'

    runs = sorted(d for d in os.listdir(root)
                  if d.startswith(prefix) and
                  os.path.exists(os.path.join(root, d, 'traj.csv')))
    if len(runs) < 2:
        print('traj.csv 가 2개 이상 필요합니다.')
        return

    D = {}
    for r in runs:
        t, xy = load(os.path.join(root, r, 'traj.csv'))
        D[r] = (t, xy, heading(xy))

    n = min(len(D[r][0]) for r in runs)
    t0 = D[runs[0]][0][0]

    print(f'비교 프레임 {n},  헤딩 반폭 {W} 프레임\n')

    # ---- 구간별 각도 차 ----
    print('### 구간별 헤딩 차이 (deg, 중앙값)\n')
    hdr = '| 구간 | 진행률 | bag 경과 |'
    sep = '|---|---|---|'
    pairs = [(runs[i], runs[j]) for i in range(len(runs)) for j in range(i + 1, len(runs))]
    for a, b in pairs:
        hdr += f' {a[-1]}-{b[-1]} |'
        sep += '---|'
    print(hdr)
    print(sep)

    edges = np.linspace(0, n, 21).astype(int)
    hist = {p: [] for p in pairs}
    for i in range(len(edges) - 1):
        lo, hi = edges[i], edges[i + 1]
        el = D[runs[0]][0][min(hi, n - 1)] - t0
        row = f'| {lo}~{hi} | {100*hi/n:.0f}% | {el:.0f}s |'
        for a, b in pairs:
            da = wrap180(np.degrees(D[a][2][lo:hi] - D[b][2][lo:hi]))
            v = np.nanmedian(da) if not np.all(np.isnan(da)) else np.nan
            hist[(a, b)].append(v)
            row += ' —  |' if np.isnan(v) else f' {v:+.1f} |'
        print(row)

    # ---- 변화가 큰 구간 ----
    print('\n### 구간 사이 각도 변화가 큰 곳 (2도 이상)\n')
    print('| 비교 | 구간 경계 | bag 경과 | 변화 |')
    print('|---|---|---|---|')
    found = False
    for a, b in pairs:
        v = hist[(a, b)]
        for i in range(1, len(v)):
            if np.isnan(v[i]) or np.isnan(v[i - 1]):
                continue
            dd = wrap180(np.array([v[i] - v[i - 1]]))[0]
            if abs(dd) >= 2.0:
                fr = edges[i]
                el = D[runs[0]][0][min(fr, n - 1)] - t0
                print(f'| {a[-1]}-{b[-1]} | 프레임 {fr} | {el:.0f}s | {dd:+.1f}도 |')
                found = True
    if not found:
        print('| — | — | — | 2도 이상 변화 없음 |')

    # ---- 초기 vs 최종 ----
    print('\n### 초기 10% vs 최종 10%\n')
    print('| 비교 | 초기 | 최종 | 증가분 | 해석 |')
    print('|---|---|---|---|---|')
    k = n // 10
    for a, b in pairs:
        e = np.nanmedian(wrap180(np.degrees(D[a][2][:k] - D[b][2][:k])))
        l = np.nanmedian(wrap180(np.degrees(D[a][2][-k:] - D[b][2][-k:])))
        if np.isnan(e) or np.isnan(l):
            continue
        grow = wrap180(np.array([l - e]))[0]
        if abs(e) > 3.0 and abs(grow) < abs(e) * 0.5:
            verdict = '초기화 기원'
        elif abs(grow) > abs(e) * 1.5:
            verdict = '누적 드리프트'
        else:
            verdict = '혼합'
        print(f'| {a[-1]}-{b[-1]} | {e:+.1f} | {l:+.1f} | {grow:+.1f} | {verdict} |')

    # ---- 그래프 ----
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.figure(figsize=(11, 4.5))
        for a, b in pairs:
            da = wrap180(np.degrees(D[a][2][:n] - D[b][2][:n]))
            plt.plot(da, lw=1, label=f'{a} vs {b}')
        plt.axhline(0, color='k', lw=.5)
        plt.xlabel('frame'); plt.ylabel('heading difference (deg)')
        plt.title('run-to-run heading difference')
        plt.legend(); plt.grid(alpha=.3); plt.tight_layout()
        out = os.path.join(root, 'yaw_diff.png')
        plt.savefig(out, dpi=110)
        print(f'\n그래프: {out}')
    except Exception as e:
        print(f'\n(그래프 생략: {e})')


if __name__ == '__main__':
    main()
