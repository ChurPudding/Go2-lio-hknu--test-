#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
repro_event.py — 발산이 일어난 순간 로봇이 무엇을 하고 있었나

  헤딩 분석에서 bag 22~44초(프레임 330~661)에 갈라지는 것이 확인됐다.
  그 구간의 속도·회전율·위치를 궤적에서 직접 읽는다.

  판독:
    · 급회전 중이었다        → 회전 중 스캔 정합이 약해지는 전형적 상황
    · 거의 정지해 있었다     → 관측 부족으로 자세가 미결정
    · 평범하게 직진 중이었다 → 기하 구조(통창 등)를 의심해야 함

  사용: python3 repro_event.py [repro 루트] [접두어]
"""

import os
import sys
import numpy as np

EVENT = (250, 750)     # 관심 구간 (프레임)
W = 8                  # 회전율 계산 반폭


def load(path):
    d = np.loadtxt(path, delimiter=',', skiprows=1, usecols=(0, 1, 2, 3))
    return d[:, 0], d[:, 1:4]


def kinematics(t, xyz):
    n = len(t)
    spd = np.zeros(n)
    dt = np.gradient(t)
    dt[dt <= 0] = 1e-3
    d = np.gradient(xyz, axis=0)
    spd = np.linalg.norm(d, axis=1) / dt

    ang = np.full(n, np.nan)
    for k in range(W, n - W):
        v = xyz[k + W, :2] - xyz[k - W, :2]
        if np.linalg.norm(v) >= 0.03:
            ang[k] = np.arctan2(v[1], v[0])
    rate = np.full(n, np.nan)
    for k in range(1, n - 1):
        if not (np.isnan(ang[k + 1]) or np.isnan(ang[k - 1])):
            da = np.degrees(np.arctan2(np.sin(ang[k + 1] - ang[k - 1]),
                                       np.cos(ang[k + 1] - ang[k - 1])))
            rate[k] = da / (t[k + 1] - t[k - 1] + 1e-6)
    return spd, ang, rate


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fastlio_ws/results/repro')
    prefix = sys.argv[2] if len(sys.argv) > 2 else 'base_'

    runs = sorted(d for d in os.listdir(root)
                  if d.startswith(prefix) and
                  os.path.exists(os.path.join(root, d, 'traj.csv')))
    if not runs:
        print('traj.csv 없음')
        return

    D = {}
    for r in runs:
        t, xyz = load(os.path.join(root, r, 'traj.csv'))
        spd, ang, rate = kinematics(t, xyz)
        D[r] = dict(t=t, xyz=xyz, spd=spd, ang=ang, rate=rate)

    ref = runs[1] if len(runs) > 1 else runs[0]   # 합의된 쪽(base_2)을 기준으로
    t0 = D[ref]['t'][0]
    a, b = EVENT

    # ---- 사건 구간 상세 ----
    print(f'### 사건 구간 상세 — 기준 {ref}\n')
    print('| 프레임 | bag 경과 | 속도(m/s) | 회전율(deg/s) | x | y | z |')
    print('|---|---|---|---|---|---|---|')
    for k in range(a, min(b, len(D[ref]['t'])), 25):
        d = D[ref]
        rt = d['rate'][k]
        print(f"| {k} | {d['t'][k]-t0:.0f}s | {d['spd'][k]:.2f} | "
              f"{'—' if np.isnan(rt) else f'{rt:+.0f}'} | "
              f"{d['xyz'][k,0]:.1f} | {d['xyz'][k,1]:.1f} | {d['xyz'][k,2]:.2f} |")

    # ---- 구간 누적 회전 ----
    print('\n### 프레임 330~661 구간의 누적 회전량 (실행별)\n')
    print('| 실행 | 시작 헤딩 | 끝 헤딩 | 누적 변화 | 평균 속도 |')
    print('|---|---|---|---|---|')
    for r in runs:
        d = D[r]
        s = np.degrees(np.nanmedian(d['ang'][330:360]))
        e = np.degrees(np.nanmedian(d['ang'][631:661]))
        tot = np.degrees(np.arctan2(np.sin(np.radians(e - s)),
                                    np.cos(np.radians(e - s))))
        print(f"| {r} | {s:+.1f} | {e:+.1f} | {tot:+.1f} | "
              f"{np.nanmean(d['spd'][330:661]):.2f} |")

    # ---- 전체에서 가장 급한 회전 ----
    print('\n### 전체 궤적에서 회전율 상위 10 (기준 실행)\n')
    d = D[ref]
    rr = np.nan_to_num(np.abs(d['rate']), nan=0.0)
    idx = np.argsort(rr)[::-1]
    picked, shown = [], 0
    print('| 순위 | 프레임 | bag 경과 | 회전율 | 속도 |')
    print('|---|---|---|---|---|')
    for k in idx:
        if any(abs(k - p) < 100 for p in picked):
            continue
        picked.append(k)
        shown += 1
        mark = '  ← 사건 구간' if a <= k <= b else ''
        print(f"| {shown} | {k} | {d['t'][k]-t0:.0f}s | "
              f"{d['rate'][k]:+.0f} deg/s | {d['spd'][k]:.2f}{mark} |")
        if shown >= 10:
            break

    # ---- 정지 구간 ----
    print('\n### 저속(<0.05 m/s) 구간 상위 5\n')
    slow = d['spd'] < 0.05
    runs_slow, s = [], None
    for k in range(len(slow)):
        if slow[k] and s is None:
            s = k
        elif not slow[k] and s is not None:
            runs_slow.append((s, k))
            s = None
    if s is not None:
        runs_slow.append((s, len(slow)))
    runs_slow.sort(key=lambda x: x[1] - x[0], reverse=True)
    print('| 프레임 구간 | bag 경과 | 길이(프레임) |')
    print('|---|---|---|')
    for st, en in runs_slow[:5]:
        mark = '  ← 사건 구간' if a <= st <= b else ''
        print(f"| {st}~{en} | {d['t'][st]-t0:.0f}s | {en-st}{mark} |")


if __name__ == '__main__':
    main()
