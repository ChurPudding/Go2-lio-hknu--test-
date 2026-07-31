#!/usr/bin/env python3
"""
compare_lio_gps.py  --  두 LIO 결과를 GPS 기준으로 한 그림에 겹쳐 비교

**공통 기준은 bag 전체 시간이다.** 특정 구간만 잘라 비교하면 어느 알고리즘에
유리한 구간을 고른 셈이 되므로, 같은 bag 을 돌린 결과는 전 구간으로 본다.

오차는 알고리즘 간 240배까지 벌어지므로 **로그 축**으로 그린다.
궤적은 GPS 영역이 보이도록 축을 제한하고, 벗어난 궤적은 화살표로 방향만 표시한다.

입력: legodom_vs_gps.py / 수동 분석이 만든 CSV
      (t, gps_x, gps_y, leg_x, leg_y, err_m, hdop)

사용
    python3 compare_lio_gps.py out.png \\
        "Point-LIO" exp/outdoor_lio_vs_gps.csv \\
        "FAST-LIO"  exp/outdoor_fastlio_vs_gps.csv
"""
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

COLORS = ['#d62728', '#2ca02c', '#9467bd', '#ff7f0e']


def setup_font():
    want = ['NanumGothic', 'Noto Sans CJK KR', 'Noto Sans KR', 'Malgun Gothic']
    have = {f.name for f in font_manager.fontManager.ttflist}
    for w in want:
        if w in have:
            plt.rcParams['font.family'] = w
            plt.rcParams['axes.unicode_minus'] = False
            return True
    return False


def load(path):
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r['t']) for r in rows])
    gx = np.array([float(r['gps_x']) for r in rows])
    gy = np.array([float(r['gps_y']) for r in rows])
    lx = np.array([float(r['leg_x']) for r in rows])
    ly = np.array([float(r['leg_y']) for r in rows])
    e = np.array([float(r['err_m']) for r in rows])
    return t, gx, gy, lx, ly, e


def main():
    ko = setup_font()
    out = sys.argv[1]
    args = sys.argv[2:]
    if len(args) < 2 or len(args) % 2:
        sys.exit(__doc__)
    sets = [(args[i], args[i + 1]) for i in range(0, len(args), 2)]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6.8),
                                   gridspec_kw={'width_ratios': [1.1, 1]})

    gps_done = False
    span = 0.0
    for i, (name, path) in enumerate(sets):
        t, gx, gy, lx, ly, e = load(path)
        c = COLORS[i % len(COLORS)]

        if not gps_done:
            ax1.plot(gx, gy, '-', lw=2.4, color='#1f77b4',
                     label='GPS (기준)' if ko else 'GPS (reference)', zorder=3)
            ax1.plot(gx[0], gy[0], 'o', ms=11, mfc='white', mec='black', mew=1.8,
                     label='시작' if ko else 'start', zorder=6)
            span = max(np.ptp(gx), np.ptp(gy))
            gps_done = True

        ax1.plot(lx, ly, '--', lw=1.9, color=c,
                 label='%s  (%.0f 초, 최종 %.0f m)' % (name, t[-1], e[-1])
                 if ko else '%s (%.0fs, final %.0fm)' % (name, t[-1], e[-1]))
        ax1.plot(lx[-1], ly[-1], 's', ms=9, color=c, zorder=5)

        ax2.plot(t, e, '-', lw=2.0, color=c, label=name)

    # 궤적 축은 GPS 영역 기준으로 제한 (벗어난 것은 잘림 = 그 자체가 결과)
    lim = span * 0.85
    cx = (ax1.dataLim.x0 + ax1.dataLim.x1) * 0
    ax1.set_xlim(-lim * 0.65, lim * 0.95)
    ax1.set_ylim(-lim * 1.05, lim * 0.35)
    ax1.set_aspect('equal', 'box')
    ax1.set_xlabel('동 [m]' if ko else 'East [m]')
    ax1.set_ylabel('북 [m]' if ko else 'North [m]')
    ax1.set_title('궤적 — bag 전 구간 공통 기준'
                  if ko else 'Trajectory - full bag timeline', fontsize=13)
    ax1.grid(alpha=0.3)
    ax1.legend(loc='lower left', fontsize=9, framealpha=0.92)
    ax1.text(0.98, 0.98, '축 밖으로 나간 궤적은 잘림' if ko else 'clipped if out of range',
             transform=ax1.transAxes, ha='right', va='top', fontsize=8, color='0.45')

    ax2.set_yscale('log')
    ax2.axhline(2.4, ls=':', lw=1.6, color='#1f77b4')
    ax2.text(0.01, 2.4, ' GPS 자체 오차 2.4 m' if ko else ' GPS own error 2.4 m',
             transform=ax2.get_yaxis_transform(), va='bottom',
             fontsize=9, color='#1f77b4')
    ax2.set_xlabel('시간 [s]' if ko else 'Time [s]')
    ax2.set_ylabel('수평 오차 [m], 로그축' if ko else 'Horizontal error [m], log')
    ax2.set_title('오차 추이 — 로그 축' if ko else 'Error over time (log)', fontsize=13)
    ax2.grid(alpha=0.3, which='both')
    ax2.legend(fontsize=10)

    fig.suptitle('실외 306 m 주행 — 알고리즘별 위치추정 (GPS 기준)'
                 if ko else 'Outdoor 306 m run - LIO vs GPS',
                 fontsize=15, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out, dpi=150)
    print('저장 ->', out)
    for name, path in sets:
        t, gx, gy, lx, ly, e = load(path)
        print('  %-12s RMS %9.1f m  최대 %9.1f m  최종 %9.1f m  (%.0f 초)'
              % (name, float(np.sqrt((e ** 2).mean())), e.max(), e[-1], t[-1]))


if __name__ == '__main__':
    main()
