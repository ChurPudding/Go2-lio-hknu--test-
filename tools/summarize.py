#!/usr/bin/env python3
"""
summarize.py -- 반복 실행 결과의 평균·표준편차 집계

사용법:
    python3 summarize.py <원본bag.db3> <그룹명1> <CSV...> [-- <그룹명2> <CSV...>]

예:
    python3 summarize.py go2_run_full_0.db3 \\
        r05 exp/r05_run*.csv -- r10 exp/r10_run*.csv

단일 실행 비교는 무의미하다. 실행 간 변동(루프 클로저 0.65~1.57 m 관측)이
설정 차이보다 클 수 있으므로 반드시 평균±표준편차로 판정한다.
"""
import sys
import csv

import numpy as np

sys.path.insert(0, '.')
try:
    from eval_lio import load_ref, load_csv, path_len
except ImportError:
    sys.exit('eval_lio.py 를 같은 디렉터리에 두세요.')


def metrics(g, G, ct, cP):
    F = np.stack([np.interp(g, ct, cP[:, k]) for k in range(3)], 1)
    gz = G[:, 2] - G[0, 2]
    fz = F[:, 2] - F[0, 2]
    e = fz - gz
    gx, gy = G[:, 0] - G[0, 0], G[:, 1] - G[0, 1]
    px, py = F[:, 0] - F[0, 0], F[:, 1] - F[0, 1]
    a0 = np.arctan2((gx * py - gy * px).sum(), (gx * px + gy * py).sum())
    rx = np.cos(a0) * gx - np.sin(a0) * gy
    ry = np.sin(a0) * gx + np.cos(a0) * gy
    err = np.hypot(px - rx, py - ry)
    return {
        'loop': float(np.linalg.norm(F[-1, :2] - F[0, :2])),
        'path': path_len(F),
        'zrmse': 1000 * float(np.sqrt((e ** 2).mean())),
        'zmax': 1000 * float(np.abs(e).max()),
        'zfin': 1000 * float(e[-1]),
        'hrms': float(err.mean()),
        'hmax': float(err.max()),
        'yaw': float(np.degrees(a0)),
    }


LABELS = [('loop', '루프클로저 [m]', 3), ('hrms', '수평RMS [m]', 3),
          ('hmax', '수평최대 [m]', 3), ('zrmse', 'z RMSE [mm]', 1),
          ('zmax', 'z 최대 [mm]', 1), ('zfin', '최종 z [mm]', 1),
          ('path', '총거리 [m]', 2), ('yaw', '정렬 yaw [deg]', 2)]


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    db = sys.argv[1]
    rest = sys.argv[2:]

    groups = []
    cur = None
    for a in rest:
        if a == '--':
            cur = None
        elif cur is None:
            cur = (a, [])
            groups.append(cur)
        else:
            cur[1].append(a)

    rt, rP, rQ = load_ref(db)
    t0 = rt[0]
    rt = rt - t0
    g = np.arange(rt[0] + 0.5, rt[-1] - 0.5, 1 / 15.0)
    G = np.stack([np.interp(g, rt, rP[:, k]) for k in range(3)], 1)

    ref = {'loop': float(np.linalg.norm(G[-1, :2] - G[0, :2])),
           'path': path_len(G),
           'zfin': 1000 * float(G[-1, 2] - G[0, 2])}
    print('기준선 robot_odom : 루프클로저 %.3f m | 총거리 %.2f m | 최종 z %+.1f mm'
          % (ref['loop'], ref['path'], ref['zfin']))

    res = {}
    for name, paths in groups:
        rows = []
        for p in paths:
            ct, cP = load_csv(p)
            rows.append(metrics(g, G, ct - t0, cP))
        res[name] = rows
        print('  %-10s n=%d' % (name, len(rows)))

    print()
    hdr = '%-16s' % '지표'
    for name in res:
        hdr += '%22s' % name
    print(hdr)
    print('-' * len(hdr))
    for key, lab, nd in LABELS:
        line = '%-16s' % lab
        for name, rows in res.items():
            v = np.array([r[key] for r in rows])
            line += '%22s' % ('%.*f ± %.*f' % (nd, v.mean(), nd, v.std(ddof=1)
                                               if len(v) > 1 else 0))
        print(line)

    print()
    for name, rows in res.items():
        v = np.array([r['loop'] for r in rows])
        print('%-10s 루프클로저 개별값: %s   (범위 %.2f배)'
              % (name, ' '.join('%.3f' % x for x in v),
                 v.max() / v.min() if v.min() > 0 else float('nan')))

    if len(res) >= 2:
        print()
        print('판정 참고: 두 그룹의 평균 차이가 각 그룹 표준편차보다 작으면')
        print('           그 차이는 실행 간 변동에 묻힌 것이며 유의하지 않다.')


if __name__ == '__main__':
    main()
