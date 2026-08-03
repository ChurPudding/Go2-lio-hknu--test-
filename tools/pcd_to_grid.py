#!/usr/bin/env python3
"""
pcd_to_grid.py  --  LIO 가 만든 PCD 지도를 A* 용 2D 점유격자로 변환

  1. PCD 를 읽어 통계와 미리보기 그림을 낸다
  2. 지면·천장을 잘라내고 수평 투영해 격자를 만든다
  3. Nav2 표준 형식(.pgm + .yaml)과 numpy(.npy)로 저장한다

지면·천장 제거가 핵심이다. 지면 점을 그대로 두면 격자가 전부 장애물이 된다.
로봇이 지나갈 수 있는 높이대(기본 0.15~1.5 m)의 점만 장애물로 본다.

Open3D 없이 동작한다 (PCD 를 직접 파싱).

사용
    python3 pcd_to_grid.py <scans.pcd> [출력접두사] [해상도m]

예)
    python3 pcd_to_grid.py ~/catkin_point_lio_unilidar/src/point_lio_ros2/PCD/scans.pcd \\
        ~/fastlio_ws/results/corridor_map 0.05
"""
import os
import re
import struct
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 로봇이 통과 가능한 높이대 [m] — 이 범위의 점만 장애물로 본다
Z_MIN, Z_MAX = 0.20, 1.50
# 한 칸에 이만큼 이상 점이 있어야 장애물로 친다 (잡음 제거)
MIN_PTS = 2


def read_pcd(path):
    """PCD (ascii / binary / binary_compressed 일부) 를 읽어 Nx3 배열 반환."""
    with open(path, 'rb') as f:
        header = b''
        while b'DATA' not in header.split(b'\n')[-2:][0] if False else True:
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

        if data == 'ascii':
            arr = np.loadtxt(f, usecols=(0, 1, 2), max_rows=npts)
            return arr.astype(np.float64)

        if data != 'binary':
            sys.exit('지원하지 않는 DATA 형식: %s' % data)

        tmap = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2',
                ('U', 4): 'u4', ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
        dt = []
        for nm, sz, tp, ct in zip(fields, size, types, count):
            base = tmap.get((tp, sz))
            if base is None:
                sys.exit('알 수 없는 필드 타입: %s %s%d' % (nm, tp, sz))
            dt.append((nm, base, ct) if ct > 1 else (nm, base))
        raw = np.frombuffer(f.read(npts * np.dtype(dt).itemsize), dtype=np.dtype(dt), count=npts)
        return np.stack([raw['x'], raw['y'], raw['z']], 1).astype(np.float64)


def main():
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else 'map'
    res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05

    P = read_pcd(src)
    P = P[np.isfinite(P).all(1)]
    print('점 %d 개' % len(P))
    print('  x %.2f ~ %.2f m   (%.2f m)' % (P[:, 0].min(), P[:, 0].max(), np.ptp(P[:, 0])))
    print('  y %.2f ~ %.2f m   (%.2f m)' % (P[:, 1].min(), P[:, 1].max(), np.ptp(P[:, 1])))
    print('  z %.2f ~ %.2f m   (%.2f m)' % (P[:, 2].min(), P[:, 2].max(), np.ptp(P[:, 2])))

    # --- 높이 분포 (지면·천장이 어디인지 확인) ---------------------------
    hist, edges = np.histogram(P[:, 2], bins=40)
    print()
    print('높이 분포 (지면·천장 확인용)')
    mx = hist.max()
    for i in range(len(hist)):
        if hist[i] < mx * 0.01:
            continue
        print('  %+6.2f ~ %+6.2f m  %8d  %s'
              % (edges[i], edges[i + 1], hist[i], '#' * int(50 * hist[i] / mx)))

    # --- 지면 기준 잡기: 최빈 높이를 지면으로 --------------------------
    ground = edges[np.argmax(hist)]
    print()
    print('추정 지면 높이 %.2f m  -> 장애물 높이대 %.2f ~ %.2f m'
          % (ground, ground + Z_MIN, ground + Z_MAX))

    m = (P[:, 2] > ground + Z_MIN) & (P[:, 2] < ground + Z_MAX)
    Q = P[m]
    print('장애물 후보 %d 점 (%.1f%%)' % (len(Q), 100 * len(Q) / len(P)))
    if len(Q) < 100:
        sys.exit('장애물 점이 너무 적다. Z_MIN/Z_MAX 를 조정할 것')

    # --- 격자화 ---------------------------------------------------------
    x0, y0 = Q[:, 0].min() - 1.0, Q[:, 1].min() - 1.0
    x1, y1 = Q[:, 0].max() + 1.0, Q[:, 1].max() + 1.0
    W = int(np.ceil((x1 - x0) / res))
    H = int(np.ceil((y1 - y0) / res))
    ix = ((Q[:, 0] - x0) / res).astype(int).clip(0, W - 1)
    iy = ((Q[:, 1] - y0) / res).astype(int).clip(0, H - 1)
    cnt = np.zeros((H, W), np.int32)
    np.add.at(cnt, (iy, ix), 1)
    occ = cnt >= MIN_PTS

    print()
    print('격자 %d x %d  (해상도 %.3f m, 원점 %.2f, %.2f)' % (W, H, res, x0, y0))
    print('  장애물 칸 %d (%.1f%%)' % (occ.sum(), 100 * occ.mean()))

    os.makedirs(os.path.dirname(os.path.abspath(out)) or '.', exist_ok=True)

    # --- npy ------------------------------------------------------------
    np.save(out + '.npy', occ.astype(np.uint8))

    # --- Nav2 표준 pgm + yaml --------------------------------------------
    # 0=장애물(검정), 254=자유(흰색). y 축은 위아래 뒤집어 저장한다.
    img = np.where(occ, 0, 254).astype(np.uint8)[::-1]
    with open(out + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H))
        f.write(img.tobytes())
    with open(out + '.yaml', 'w') as f:
        f.write('image: %s.pgm\n' % os.path.basename(out))
        f.write('resolution: %.4f\n' % res)
        f.write('origin: [%.4f, %.4f, 0.0]\n' % (x0, y0))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

    # --- 미리보기 그림 ---------------------------------------------------
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 7))
    s = max(1, len(P) // 300000)
    a1.scatter(P[::s, 0], P[::s, 1], s=0.2, c=P[::s, 2], cmap='viridis', linewidths=0)
    a1.set_aspect('equal'); a1.grid(alpha=0.3)
    a1.set_title('PCD top view (all points, color=height)')
    a1.set_xlabel('x [m]'); a1.set_ylabel('y [m]')

    a2.imshow(occ, origin='lower', cmap='gray_r',
              extent=[x0, x0 + W * res, y0, y0 + H * res])
    a2.set_aspect('equal'); a2.grid(alpha=0.3)
    a2.set_title('occupancy grid  %.0f cm  (%d x %d)' % (res * 100, W, H))
    a2.set_xlabel('x [m]'); a2.set_ylabel('y [m]')
    fig.tight_layout()
    fig.savefig(out + '_preview.png', dpi=140)

    print()
    print('저장')
    for e in ('.npy', '.pgm', '.yaml', '_preview.png'):
        print('  %s%s' % (out, e))


if __name__ == '__main__':
    main()
