#!/usr/bin/env python3
"""
plot_legodom_gps.py  --  다리 오도메트리 vs GPS 궤적 겹쳐 그리기

legodom_vs_gps.py 가 만든 CSV 를 읽어 보고용 그림 2장을 만든다.

  (좌) 궤적 겹침 — GPS 실선, 다리 오도메트리 파선, 오차 큰 지점 표시
  (우) 시간에 따른 수평 오차

matplotlib 이 없으면:
    pip3 install matplotlib --break-system-packages

사용
    python3 plot_legodom_gps.py legodom_vs_gps.csv [출력.png]
"""
import csv
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager


def setup_font():
    """한글 폰트가 있으면 쓰고, 없으면 라벨을 영문으로 바꾼다."""
    want = ['NanumGothic', 'Noto Sans CJK KR', 'Noto Sans KR',
            'Malgun Gothic', 'AppleGothic', 'UnDotum']
    have = {f.name for f in font_manager.fontManager.ttflist}
    for w in want:
        if w in have:
            plt.rcParams['font.family'] = w
            plt.rcParams['axes.unicode_minus'] = False
            return True
    print('한글 폰트 없음 -> 영문 라벨 사용')
    print('  한글로 보려면: sudo apt install -y fonts-nanum && rm -rf ~/.cache/matplotlib')
    return False


KO = {
    'gps': 'GPS (기준)', 'leg': '다리 오도메트리', 'start': '시작',
    'east': '동 [m]', 'north': '북 [m]', 'time': '시간 [s]',
    'err': '수평 오차 [m]',
    'title1': '실외 356 m 주행 — GPS vs 다리 오도메트리',
    'maxerr': '최대 오차 %.1f m\n(t=%.0f s)',
    'gpserr': ' GPS 자체 오차 2.4 m',
    'title2': '오차 추이 — 최대 %.1f m, 최종 %.1f m',
    'sup': '다리 운동학 오도메트리의 실외 표류 (GPS 기준)',
}
EN = {
    'gps': 'GPS (reference)', 'leg': 'Leg odometry', 'start': 'start',
    'east': 'East [m]', 'north': 'North [m]', 'time': 'Time [s]',
    'err': 'Horizontal error [m]',
    'title1': 'Outdoor 356 m run - GPS vs leg odometry',
    'maxerr': 'max error %.1f m\n(t=%.0f s)',
    'gpserr': ' GPS own error 2.4 m',
    'title2': 'Error over time - max %.1f m, final %.1f m',
    'sup': 'Outdoor drift of leg kinematic odometry (GPS reference)',
}


def main():
    L = KO if setup_font() else EN
    src = sys.argv[1] if len(sys.argv) > 1 else 'legodom_vs_gps.csv'
    out = sys.argv[2] if len(sys.argv) > 2 else 'legodom_vs_gps.png'

    rows = list(csv.DictReader(open(src)))
    t = np.array([float(r['t']) for r in rows])
    gx = np.array([float(r['gps_x']) for r in rows])
    gy = np.array([float(r['gps_y']) for r in rows])
    lx = np.array([float(r['leg_x']) for r in rows])
    ly = np.array([float(r['leg_y']) for r in rows])
    e = np.array([float(r['err_m']) for r in rows])

    imax = int(np.argmax(e))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6.5),
                                   gridspec_kw={'width_ratios': [1.15, 1]})

    # ---------------- 좌: 궤적 -----------------------------------------
    ax1.plot(gx, gy, '-', lw=2.0, color='#1f77b4', label=L['gps'])
    ax1.plot(lx, ly, '--', lw=1.8, color='#d62728', label=L['leg'])

    # 대응점을 잇는 얇은 선 — 벌어진 정도가 한눈에 보인다
    step = max(1, len(t) // 60)
    for i in range(0, len(t), step):
        ax1.plot([gx[i], lx[i]], [gy[i], ly[i]],
                 '-', color='0.75', lw=0.6, zorder=0)

    ax1.plot(gx[0], gy[0], 'o', ms=10, mfc='white', mec='black',
             mew=1.6, label=L['start'], zorder=5)
    ax1.plot(gx[-1], gy[-1], 's', ms=9, color='#1f77b4', zorder=5)
    ax1.plot(lx[-1], ly[-1], 's', ms=9, color='#d62728', zorder=5)

    ax1.plot([gx[imax], lx[imax]], [gy[imax], ly[imax]],
             '-', color='black', lw=1.8, zorder=6)
    ax1.annotate(L['maxerr'] % (e[imax], t[imax]),
                 xy=((gx[imax] + lx[imax]) / 2, (gy[imax] + ly[imax]) / 2),
                 xytext=(12, 12), textcoords='offset points',
                 fontsize=11, fontweight='bold',
                 bbox=dict(boxstyle='round,pad=0.4', fc='#fff3cd', ec='0.4'))

    ax1.set_aspect('equal', 'datalim')
    ax1.set_xlabel(L['east'])
    ax1.set_ylabel(L['north'])
    ax1.set_title(L['title1'], fontsize=13)
    ax1.grid(alpha=0.3)
    ax1.legend(loc='best', framealpha=0.9)

    # ---------------- 우: 오차 -----------------------------------------
    ax2.plot(t, e, '-', lw=1.8, color='#d62728')
    ax2.fill_between(t, 0, e, color='#d62728', alpha=0.15)
    ax2.axhline(2.4, ls=':', color='#1f77b4', lw=1.6)
    ax2.text(t[-1], 2.4, L['gpserr'], va='bottom', ha='right',
             fontsize=9, color='#1f77b4')

    rms = float(np.sqrt((e ** 2).mean()))
    ax2.axhline(rms, ls='--', color='0.4', lw=1.2)
    ax2.text(t[0], rms, ' RMS %.1f m' % rms, va='bottom', fontsize=9, color='0.3')

    ax2.plot(t[imax], e[imax], 'o', ms=8, color='black')
    ax2.set_xlabel(L['time'])
    ax2.set_ylabel(L['err'])
    ax2.set_title(L['title2'] % (e.max(), e[-1]),
                  fontsize=13)
    ax2.grid(alpha=0.3)
    ax2.set_ylim(0, max(e.max() * 1.15, 5))

    fig.suptitle(L['sup'],
                 fontsize=15, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=150)
    print('저장 ->', out)
    print('  RMS %.1f m  최대 %.1f m (t=%.0f s)  최종 %.1f m'
          % (rms, e.max(), t[imax], e[-1]))


if __name__ == '__main__':
    main()
