#!/usr/bin/env python3
"""
pcd_view.py  --  PCD 점군을 여러 각도에서 본 그림으로 만든다

    python3 pcd_view.py <scans.pcd> [출력접두사] [최대점수]

예)
    python3 pcd_view.py \\
      ~/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/scans.pcd \\
      results/pcd_view 400000

Open3D 없이 matplotlib 로 그린다. PCD 는 직접 파싱한다.

큰 파일 다루기
------------
348 MB 면 약 900 만 점이다. 전부 그리면 느리고 그림도 뭉개진다.
무작위로 골라 max_points(기본 40만) 개만 그린다. 형태를 보는 데는 충분하다.

만드는 그림
---------
    _top.png     위에서 본 모습 (x-y). 방 모양 확인
    _front.png   앞에서 본 모습 (x-z). 층·기울기 확인
    _side.png    옆에서 본 모습 (y-z)
    _hist.png    높이 분포. 봉우리 개수로 지도 품질 판정

**판정 기준**
    z 범위 4 m 이내, 높이 봉우리 1 개  → 정상
    z 범위 10 m 이상, 봉우리 여러 개    → 발산했거나 이전 회차가 섞임
"""
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_pcd(path, max_points=None):
    """PCD (ascii / binary) 를 읽어 Nx3. 큰 파일은 골라서 읽는다."""
    with open(path, 'rb') as f:
        header = b''
        while True:
            line = f.readline()
            if not line:
                break
            header += line
            if line.startswith(b'DATA'):
                break
        txt = header.decode('ascii', 'replace')
        fields = re.search(r'FIELDS (.+)', txt).group(1).split()
        size = [int(x) for x in re.search(r'SIZE (.+)', txt).group(1).split()]
        types = re.search(r'TYPE (.+)', txt).group(1).split()
        count = [int(x) for x in re.search(r'COUNT (.+)', txt).group(1).split()]
        npts = int(re.search(r'POINTS (\d+)', txt).group(1))
        data = re.search(r'DATA (\S+)', txt).group(1)

        print('점 %d 개, 형식 %s' % (npts, data))

        if data == 'ascii':
            arr = np.loadtxt(f, usecols=(0, 1, 2), max_rows=npts)
            P = arr.astype(np.float64)
        elif data == 'binary':
            tmap = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2',
                    ('U', 4): 'u4', ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
            dt = []
            for nm, sz, tp, ct in zip(fields, size, types, count):
                base = tmap.get((tp, sz))
                if base is None:
                    sys.exit('알 수 없는 필드: %s %s%d' % (nm, tp, sz))
                dt.append((nm, base, ct) if ct > 1 else (nm, base))
            dt = np.dtype(dt)
            raw = np.frombuffer(f.read(npts * dt.itemsize), dtype=dt, count=npts)
            P = np.stack([raw['x'], raw['y'], raw['z']], 1).astype(np.float64)
        else:
            sys.exit('지원하지 않는 DATA 형식: %s' % data)

    P = P[np.isfinite(P).all(1)]
    if max_points and len(P) > max_points:
        idx = np.random.default_rng(0).choice(len(P), max_points, replace=False)
        P = P[idx]
        print('  -> %d 개만 골라서 그린다' % len(P))
    return P


def scatter(ax, a, b, c, title, xlabel, ylabel):
    s = ax.scatter(a, b, s=0.15, c=c, cmap='viridis', linewidths=0)
    ax.set_aspect('equal')
    ax.grid(alpha=0.25)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    return s


def main():
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else 'pcd_view'
    maxp = int(sys.argv[3]) if len(sys.argv) > 3 else 400000

    print('%s  (%.0f MB)' % (src, os.path.getsize(src) / 1e6))
    P = read_pcd(src, maxp)

    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    print()
    print('  x %.2f ~ %.2f m   (%.2f m)' % (x.min(), x.max(), np.ptp(x)))
    print('  y %.2f ~ %.2f m   (%.2f m)' % (y.min(), y.max(), np.ptp(y)))
    print('  z %.2f ~ %.2f m   (%.2f m)' % (z.min(), z.max(), np.ptp(z)))

    os.makedirs(os.path.dirname(os.path.abspath(out)) or '.', exist_ok=True)

    # ── 위에서 본 모습 ────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(11, 11))
    sc = scatter(ax, x, y, z, 'top view (x-y), color = height', 'x [m]', 'y [m]')
    fig.colorbar(sc, ax=ax, shrink=0.6, label='z [m]')
    fig.tight_layout(); fig.savefig(out + '_top.png', dpi=140); plt.close(fig)

    # ── 앞에서 본 모습 ────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(13, 6))
    scatter(ax, x, z, z, 'front view (x-z)', 'x [m]', 'z [m]')
    fig.tight_layout(); fig.savefig(out + '_front.png', dpi=140); plt.close(fig)

    # ── 옆에서 본 모습 ────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(13, 6))
    scatter(ax, y, z, z, 'side view (y-z)', 'y [m]', 'z [m]')
    fig.tight_layout(); fig.savefig(out + '_side.png', dpi=140); plt.close(fig)

    # ── 높이 분포 ─────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(9, 5))
    hist, edges, _ = ax.hist(z, bins=120, color='#1D9E75')
    ax.set_xlabel('z [m]'); ax.set_ylabel('count')
    ax.set_title('height distribution')
    ax.grid(alpha=0.25)
    fig.tight_layout(); fig.savefig(out + '_hist.png', dpi=140); plt.close(fig)

    # ── 판정 ──────────────────────────────────────────────────────────
    ground = edges[int(np.argmax(hist))]
    peaks = int((hist > hist.max() * 0.15).sum())
    zr = float(np.ptp(z))
    print()
    print('추정 지면 %.2f m,  큰 봉우리 %d 개' % (ground, peaks))
    print()
    if zr < 4.0:
        print('  z 범위 %.2f m  → 정상' % zr)
    else:
        print('  z 범위 %.2f m  → ⚠ 발산했거나 이전 회차가 섞였을 수 있다' % zr)
        print('     PCD 를 지우고 다시 만들어 볼 것')

    print()
    print('저장')
    for e in ('_top.png', '_front.png', '_side.png', '_hist.png'):
        print('  %s%s' % (out, e))


if __name__ == '__main__':
    main()
