#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
repro_report.py — 반복 실행 결과를 한 표로 모은다

  사용: python3 repro_report.py [repro 루트, 기본 ~/fastlio_ws/results/repro] [접두어]

  핵심은 두 줄이다:
    · 처리 프레임 수  → 3회가 같으면 "입력은 같았다"
    · 시작-끝 거리    → 폐곡선이므로 곧 누적 드리프트

  지도 범위는 min/max 대신 1~99 퍼센타일을 쓴다.
  min/max 는 이상점 하나에 통째로 흔들려 재현성의 자로 쓸 수 없다.
"""

import json
import os
import re
import sys
import statistics

import numpy as np


def load_pcd_xyz(path):
    """open3d 우선, 없으면 ascii/binary PCD 를 직접 읽는다."""
    try:
        import open3d as o3d
        pc = o3d.io.read_point_cloud(path)
        pts = np.asarray(pc.points)
        if pts.size:
            return pts
    except Exception:
        pass

    with open(path, 'rb') as f:
        fields, sizes, types, counts = [], [], [], []
        npoints, data_type = 0, 'ascii'
        while True:
            line = f.readline()
            if not line:
                return None
            s = line.decode('ascii', 'ignore').strip()
            if s.startswith('FIELDS'):
                fields = s.split()[1:]
            elif s.startswith('SIZE'):
                sizes = [int(v) for v in s.split()[1:]]
            elif s.startswith('TYPE'):
                types = s.split()[1:]
            elif s.startswith('COUNT'):
                counts = [int(v) for v in s.split()[1:]]
            elif s.startswith('POINTS'):
                npoints = int(s.split()[1])
            elif s.startswith('DATA'):
                data_type = s.split()[1]
                break

        if not counts:
            counts = [1] * len(fields)

        if data_type == 'ascii':
            arr = np.loadtxt(f, usecols=(0, 1, 2), max_rows=npoints)
            return arr.reshape(-1, 3)

        if data_type == 'binary':
            np_types = {('F', 4): 'f4', ('F', 8): 'f8',
                        ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4',
                        ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
            dtype = []
            for name, t, sz, cnt in zip(fields, types, sizes, counts):
                dtype.append((name, np_types[(t, sz)], cnt) if cnt > 1
                             else (name, np_types[(t, sz)]))
            data = np.frombuffer(f.read(), dtype=np.dtype(dtype), count=npoints)
            return np.stack([data['x'], data['y'], data['z']], axis=1).astype(float)

        return None   # binary_compressed 는 open3d 필요


def pct_range(v, lo=1, hi=99):
    return float(np.percentile(v, hi) - np.percentile(v, lo))


def analyze(run_dir):
    r = {'name': os.path.basename(run_dir)}

    mj = os.path.join(run_dir, 'monitor.json')
    if os.path.exists(mj):
        with open(mj) as f:
            r.update(json.load(f))

    sig = os.path.join(run_dir, 'signals.txt')
    if os.path.exists(sig):
        txt = open(sig, encoding='utf-8', errors='ignore').read()
        pcts = [float(x) for x in re.findall(r'IMU Initializing:\s*([\d.]+)\s*%', txt)]
        r['imu_init_steps'] = len(pcts)
        r['imu_init_seq'] = '→'.join(f'{p:g}' for p in pcts[:8])
        # 1% 다음이 곧바로 100% 면 점프 = 초기화 실패
        r['imu_init_ok'] = not any(
            pcts[i] < 5 and pcts[i + 1] > 95 for i in range(len(pcts) - 1))
        blocks = txt.split('###')
        for b in blocks:
            if 'imu loop back' in b:
                m = re.search(r'\n\s*(\d+)', b)
                r['loop_back'] = int(m.group(1)) if m else None
            if 'No Effective Points' in b:
                m = re.search(r'\n\s*(\d+)', b)
                r['no_eff_points'] = int(m.group(1)) if m else None

    pcd = os.path.join(run_dir, 'scans.pcd')
    if os.path.exists(pcd):
        pts = load_pcd_xyz(pcd)
        if pts is not None and len(pts):
            r['pcd_points'] = int(len(pts))
            r['x_p99'] = round(pct_range(pts[:, 0]), 2)
            r['y_p99'] = round(pct_range(pts[:, 1]), 2)
            r['z_p99'] = round(pct_range(pts[:, 2]), 2)
            r['x_minmax'] = round(float(pts[:, 0].max() - pts[:, 0].min()), 2)
            r['y_minmax'] = round(float(pts[:, 1].max() - pts[:, 1].min()), 2)
            hist, edges = np.histogram(pts[:, 2], bins=100)
            k = int(np.argmax(hist))
            r['ground_z'] = round(float((edges[k] + edges[k + 1]) / 2), 3)
    return r


ROWS = [
    ('처리 프레임 수 (/aft_mapped_to_init)', 'odom_frames', '{}'),
    ('★ 시작-끝 거리 xy (m)', 'start_end_dist_xy_m', '{}'),
    ('경로 길이 (m)', 'path_length_m', '{}'),
    ('IMU 초기화 정상', 'imu_init_ok', '{}'),
    ('IMU 초기화 단계', 'imu_init_seq', '{}'),
    ('imu loop back', 'loop_back', '{}'),
    ('No Effective Points', 'no_eff_points', '{}'),
    ('PCD 점 개수', 'pcd_points', '{}'),
    ('x 범위 p1~p99 (m)', 'x_p99', '{}'),
    ('y 범위 p1~p99 (m)', 'y_p99', '{}'),
    ('z 퍼짐 p1~p99 (m)', 'z_p99', '{}'),
    ('x 범위 min~max (m)', 'x_minmax', '{}'),
    ('지면 추정 z (m)', 'ground_z', '{}'),
    ('CPU 평균 (%)', 'cpu_mean_percent', '{}'),
    ('CPU 최대 (%)', 'cpu_max_percent', '{}'),
    ('1초 이상 정지 횟수', 'stall_count_gt_1s', '{}'),
]

KEY_METRICS = ['start_end_dist_xy_m', 'x_p99', 'y_p99', 'z_p99', 'path_length_m']


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/fastlio_ws/results/repro')
    prefix = sys.argv[2] if len(sys.argv) > 2 else ''

    dirs = sorted(d for d in os.listdir(root)
                  if os.path.isdir(os.path.join(root, d)) and d.startswith(prefix))
    if not dirs:
        print(f'결과 없음: {root} (접두어 "{prefix}")')
        return

    runs = [analyze(os.path.join(root, d)) for d in dirs]

    w = 34
    print('\n| 항목 | ' + ' | '.join(r['name'] for r in runs) + ' |')
    print('|' + '---|' * (len(runs) + 1))
    for label, key, fmt in ROWS:
        cells = []
        for r in runs:
            v = r.get(key)
            cells.append('—' if v is None else fmt.format(v))
        print(f'| {label} | ' + ' | '.join(cells) + ' |')

    print('\n### 편차')
    print('\n| 지표 | 평균 | 표준편차 | 최대-최소 | 변동계수 |')
    print('|---|---|---|---|---|')
    for key in KEY_METRICS:
        vals = [r[key] for r in runs if isinstance(r.get(key), (int, float))]
        if len(vals) < 2:
            continue
        m = statistics.mean(vals)
        sd = statistics.stdev(vals)
        cv = (sd / m * 100) if m else float('nan')
        print(f'| {key} | {m:.3f} | {sd:.3f} | {max(vals)-min(vals):.3f} | {cv:.1f}% |')

    # ---- 판정 트리 ----
    print('\n### 판정')
    frames = [r.get('odom_frames') for r in runs if r.get('odom_frames')]
    if len(set(frames)) == 1 and frames:
        print(f'· 처리 프레임 수가 {frames[0]} 로 3회 동일 → 입력·처리량은 같았음.')
        print('  드롭 가설(b) 기각. 편차가 크면 원인은 알고리즘 내부 비결정성.')
    elif frames:
        print(f'· 처리 프레임 수가 다름: {frames}')
        print('  ★ 매 실행 실제 입력이 달랐다는 뜻. -r 0.25 로 낮춰 재측정할 것.')

    se = [r.get('start_end_dist_xy_m') for r in runs
          if isinstance(r.get('start_end_dist_xy_m'), (int, float))]
    if len(se) >= 2:
        sd = statistics.stdev(se)
        print(f'· 시작-끝 거리: {[round(v,2) for v in se]} (표준편차 {sd:.2f} m)')
        if sd < 1.0:
            print('  → 재현성 확보. 다음 단계(SimpleLoopClosure) 진행 가능.')
        else:
            print('  → 재현성 미확보. 비교 실험은 아직 판정 불가.')

    out = os.path.join(root, f'report_{prefix or "all"}.json')
    with open(out, 'w') as f:
        json.dump(runs, f, indent=2, ensure_ascii=False)
    print(f'\n원자료: {out}')


if __name__ == '__main__':
    main()
