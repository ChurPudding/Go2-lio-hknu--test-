#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
repro_diverge.py — 3회 실행이 '언제부터' 갈라지는지 찾는다

  전제: 3회 모두 처리 프레임 6616 로 동일하므로, k번째 궤적점은
        3회 모두 같은 원시 측정값에 대응한다. 그래서 직접 비교할 수 있다.

  두 가지를 본다:
    1) 발산 곡선 — 프레임 순서에 따른 실행 간 거리
       · 완만히 증가       → 수치 누적 (순수 내부 비결정성)
       · 특정 지점에서 급증 → 그 구간의 기하 구조가 원인 (통창 의심)
    2) 강체 정합 잔차 — 앞부분을 Kabsch 로 맞춘 뒤 뒷부분이 맞는지
       · 잔차가 작다 → 차이는 '회전' 하나로 설명됨
       · 잔차가 크다 → 형상 자체가 달라짐

  사용: python3 repro_diverge.py [repro 루트] [접두어]
"""

import os
import sys
import numpy as np


def load_traj(path):
    d = np.loadtxt(path, delimiter=',', skiprows=1)
    return d[:, 0], d[:, 1:4]      # stamp, xyz


def kabsch_2d(P, Q):
    """P 를 Q 에 맞추는 회전(yaw)+평행이동. 반환: (yaw_deg, RMSE)"""
    Pc, Qc = P.mean(0), Q.mean(0)
    H = (P - Pc).T @ (Q - Qc)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    res = (R @ (P - Pc).T).T + Qc - Q
    return yaw, float(np.sqrt((res ** 2).sum(1).mean()))


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fastlio_ws/results/repro')
    prefix = sys.argv[2] if len(sys.argv) > 2 else 'base_'

    runs = sorted(d for d in os.listdir(root)
                  if d.startswith(prefix) and
                  os.path.exists(os.path.join(root, d, 'traj.csv')))
    if len(runs) < 2:
        print('traj.csv 가 2개 이상 필요합니다.')
        return

    data = {}
    for r in runs:
        t, xyz = load_traj(os.path.join(root, r, 'traj.csv'))
        data[r] = (t, xyz)
        print(f'{r}: {len(t)} 점')

    n = min(len(v[0]) for v in data.values())
    print(f'\n공통 길이 {n} 프레임으로 비교\n')

    # ---------------- 1. 발산 곡선 ----------------
    print('### 발산 곡선 (프레임 구간별 실행 간 최대 거리)')
    print('\n| 구간 | 진행률 | 최대 이격 (m) | 구간 증가 |')
    print('|---|---|---|---|')

    edges = np.linspace(0, n, 21).astype(int)
    prev = 0.0
    jump_at = None
    for i in range(len(edges) - 1):
        a, b = edges[i], edges[i + 1]
        dmax = 0.0
        keys = list(data)
        for x in range(len(keys)):
            for y in range(x + 1, len(keys)):
                P = data[keys[x]][1][a:b]
                Q = data[keys[y]][1][a:b]
                dmax = max(dmax, float(np.linalg.norm(P - Q, axis=1).max()))
        inc = dmax - prev
        mark = ''
        if inc > 3.0 and jump_at is None:
            mark = '  ← ★ 급증'
            jump_at = (a, b)
        print(f'| {a}~{b} | {100*b/n:.0f}% | {dmax:.2f} | {inc:+.2f}{mark} |')
        prev = dmax

    # 임계 도달 시점
    print('\n### 이격이 처음 임계를 넘는 프레임')
    keys = list(data)
    P = data[keys[0]][1][:n]
    Q = data[keys[1]][1][:n]
    dist = np.linalg.norm(P - Q, axis=1)
    print(f'\n({keys[0]} vs {keys[1]})\n')
    print('| 임계 | 프레임 | 진행률 | 재생 경과 |')
    print('|---|---|---|---|')
    t0 = data[keys[0]][0][0]
    for th in (0.05, 0.2, 1.0, 5.0, 10.0, 20.0):
        idx = np.argmax(dist > th) if (dist > th).any() else None
        if idx is None:
            print(f'| {th} m | 도달 안 함 | — | — |')
        else:
            el = data[keys[0]][0][idx] - t0
            print(f'| {th} m | {idx} | {100*idx/n:.1f}% | {el:.1f} s |')

    # ---------------- 2. 강체 정합 ----------------
    print('\n### 강체 정합 — 차이가 회전 하나로 설명되는가')
    print('\n| 비교 | 구간 | yaw 차 (deg) | 정합 후 RMSE (m) |')
    print('|---|---|---|---|')
    half = n // 2
    for x in range(len(keys)):
        for y in range(x + 1, len(keys)):
            A = data[keys[x]][1][:, :2]
            B = data[keys[y]][1][:, :2]
            for label, sl in (('전체', slice(0, n)),
                              ('전반', slice(0, half)),
                              ('후반', slice(half, n))):
                yaw, rmse = kabsch_2d(A[sl], B[sl])
                print(f'| {keys[x]} vs {keys[y]} | {label} | {yaw:+.2f} | {rmse:.3f} |')

    # ---------------- 3. 그래프 ----------------
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        for x in range(len(keys)):
            for y in range(x + 1, len(keys)):
                d = np.linalg.norm(data[keys[x]][1][:n] - data[keys[y]][1][:n], axis=1)
                ax[0].plot(d, label=f'{keys[x]} vs {keys[y]}')
        ax[0].set_xlabel('frame')
        ax[0].set_ylabel('distance (m)')
        ax[0].set_title('run-to-run divergence')
        ax[0].legend(); ax[0].grid(alpha=.3)

        for k in keys:
            xyz = data[k][1]
            ax[1].plot(xyz[:, 0], xyz[:, 1], lw=1, label=k)
            ax[1].plot(xyz[0, 0], xyz[0, 1], 'ko', ms=6)
            ax[1].plot(xyz[-1, 0], xyz[-1, 1], 'rx', ms=8)
        ax[1].set_aspect('equal')
        ax[1].set_xlabel('x (m)'); ax[1].set_ylabel('y (m)')
        ax[1].set_title('trajectories (o=start, x=end)')
        ax[1].legend(); ax[1].grid(alpha=.3)

        out = os.path.join(root, 'diverge.png')
        plt.tight_layout(); plt.savefig(out, dpi=110)
        print(f'\n그래프: {out}')
    except Exception as e:
        print(f'\n(그래프 생략: {e})')


if __name__ == '__main__':
    main()
