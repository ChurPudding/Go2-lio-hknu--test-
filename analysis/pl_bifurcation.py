#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pl_bifurcation.py — Point-LIO 회차 겹쳐보기:
    뭉개짐 / yaw 분기가 '언제 · 어디서 · 무슨 동작'에서 시작되는지 찾는다.

핵심 아이디어
  같은 bag 의 여러 재생본은 입력 데이터가 동일하다. 그런데 결과가 갈린다면,
  그 갈라짐(bifurcation) 이 '언제' 시작되는지가 원인 규명의 핵심이다.
  회차마다 프레임 수가 달라(메시지 드롭) 인덱스로는 못 맞추므로,
  t_sec 을 공통 시각 축으로 삼아 선형보간해서 '같은 시각끼리' 비교한다.

무엇을 출력하나
  [1] 각 회차의 운동 상태:  속도 v(t), 각속도 ω(t), 동작 라벨(정지/보행/회전)
      → 정지 구간(예: 194~220초)이 실제로 검출되는지 자동 확인
  [2] 기준 회차 대비 Δyaw(t), Δposition(t) 와 '분기 시작 시각 t*'
      → t* 시점의 위치(x,y)와 그때 로봇이 하던 동작
  [3] 그림 4장 (matplotlib 있을 때) : 원점거리 / yaw / Δyaw / Δpos 겹쳐 그리기

판독 규칙 (예전 repro_yaw 논리 계승)
  · 초기 Δyaw 가 크다        → 초기화(정렬) 문제. yaw 는 자력계 없이 관측 불가.
  · 초기엔 0인데 t* 에서 증가 → 그 구간에 원인이 있다 (회전/정지/특정 위치).
  · 완만히 계속 증가         → 자이로 적분 드리프트 누적.

사용
  python3 pl_bifurcation.py <기준.csv> <비교1.csv> [비교2.csv ...]
  예) python3 pl_bifurcation.py run_PL_run2_b.csv run_PL_run2_a.csv \
                                run_PL_run2_c.csv run_PL_run2_d.csv

CSV 컬럼(가정): t_sec,x,y,z,qx,qy,qz,qw,yaw_deg,dist_from_origin
"""

import sys
import os
import numpy as np

# ------------------------- 조정 가능한 임계값 -------------------------
DT        = 0.1     # 공통 시각 격자 간격 [s]
V_STILL   = 0.05    # 이 속도 미만 = 정지 후보 [m/s]
W_TURN    = 15.0    # 이 각속도 이상 = 회전 [deg/s]
MIN_STILL = 3.0     # 이 시간 이상 지속돼야 '정지 구간'으로 인정 [s]
YAW_THR   = 5.0     # Δyaw 분기 판정 임계 [deg]
POS_THR   = 0.5     # Δposition 분기 판정 임계 [m]
HOLD      = 2.0     # 임계를 넘고 이 시간 이상 유지돼야 '분기 시작'으로 인정 [s]
SMOOTH    = 5       # 이동평균 창(샘플 수). 5샘플 = 0.5초
# --------------------------------------------------------------------


def smooth(x, w=SMOOTH):
    if w <= 1:
        return x
    k = np.ones(w) / w
    return np.convolve(x, k, mode='same')


def load(path):
    """t_sec, x, y, yaw_deg(감김 해제) 를 돌려준다."""
    a = np.genfromtxt(path, delimiter=',', skip_header=1)
    if a.ndim != 2 or a.shape[1] < 9:
        raise ValueError(f"{path}: 컬럼이 예상과 다름 (shape={a.shape})")
    t   = a[:, 0]
    x   = a[:, 1]
    y   = a[:, 2]
    yaw = a[:, 8]
    # yaw 는 ±180 로 감겨 있으므로 미분/보간 전에 반드시 풀어준다
    yaw_u = np.rad2deg(np.unwrap(np.deg2rad(yaw)))
    return t, x, y, yaw_u


def resample(t, v, grid):
    """t 위의 값 v 를 공통 격자 grid 로 선형보간."""
    return np.interp(grid, t, v)


def kinematics(x, y, yaw_u, dt=DT):
    """등간격 격자 위에서 속도·각속도를 중심차분으로.
         v(t) = sqrt( (dx/dt)^2 + (dy/dt)^2 )
         w(t) = d(yaw)/dt
    """
    vx = np.gradient(x, dt)
    vy = np.gradient(y, dt)
    v  = smooth(np.hypot(vx, vy))
    w  = smooth(np.gradient(yaw_u, dt))
    return v, w


def label_motion(v, w):
    """동작 라벨.
         회전 : |w| >= W_TURN                 (제자리·이동 회전 모두)
         정지 : v < V_STILL  그리고 |w| < W_TURN
         보행 : 그 외 (v >= V_STILL)
    """
    lab = np.empty(len(v), dtype=object)
    aw = np.abs(w)
    for i in range(len(v)):
        if aw[i] >= W_TURN:
            lab[i] = 'turn'
        elif v[i] < V_STILL:
            lab[i] = 'still'
        else:
            lab[i] = 'walk'
    return lab


def find_still_segments(grid, lab, min_dur=MIN_STILL):
    """연속된 'still' 구간 → (시작t, 끝t, 지속t) 리스트."""
    segs = []
    n = len(lab)
    i = 0
    while i < n:
        if lab[i] == 'still':
            j = i
            while j < n and lab[j] == 'still':
                j += 1
            dur = grid[j - 1] - grid[i]
            if dur >= min_dur:
                segs.append((grid[i], grid[j - 1], dur))
            i = j
        else:
            i += 1
    return segs


def first_sustained(grid, val, thr, hold=HOLD, dt=DT):
    """|val| 이 thr 을 넘고 hold 초 이상 계속 넘는 첫 시각. 없으면 None."""
    hn = max(1, int(round(hold / dt)))
    a = np.abs(val)
    n = len(a)
    for i in range(n - hn):
        if np.all(a[i:i + hn] > thr):
            return grid[i]
    return None


def idx_at(grid, t):
    return int(np.argmin(np.abs(grid - t)))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    paths = sys.argv[1:]
    names = [os.path.splitext(os.path.basename(p))[0] for p in paths]
    ref_name = names[0]

    raw = [load(p) for p in paths]

    # 공통 격자: 모든 회차가 공유하는 시간 구간
    t0 = max(r[0][0]  for r in raw)
    t1 = min(r[0][-1] for r in raw)
    grid = np.arange(t0, t1, DT)
    print(f"공통 시각 구간 : {t0:.1f} ~ {t1:.1f} s  ({len(grid)} 샘플, dt={DT}s)")
    print(f"기준 회차      : {ref_name}\n")

    # 격자로 보간 + 운동학 + 라벨
    R = []
    for (t, x, y, yaw_u), nm in zip(raw, names):
        xg   = resample(t, x, grid)
        yg   = resample(t, y, grid)
        yawg = resample(t, yaw_u, grid)
        v, w = kinematics(xg, yg, yawg)
        lab  = label_motion(v, w)
        R.append(dict(name=nm, x=xg, y=yg, yaw=yawg, v=v, w=w, lab=lab))
    ref = R[0]

    # ---------------- [1] 회차별 정지 구간 ----------------
    print("=" * 68)
    print("[1] 회차별 정지 구간  (v<%.2f m/s, %.0fs 이상 지속)" % (V_STILL, MIN_STILL))
    print("=" * 68)
    for r in R:
        segs = find_still_segments(grid, r['lab'])
        if not segs:
            print(f"  {r['name']:16s} : (없음)")
            continue
        for (s, e, d) in segs:
            i = idx_at(grid, (s + e) / 2)
            print(f"  {r['name']:16s} : {s:6.1f}~{e:6.1f}s ({d:4.1f}s)  "
                  f"위치 ({r['x'][i]:+.1f}, {r['y'][i]:+.1f})")
    print()

    # ---------------- [2] 기준 대비 분기 ----------------
    print("=" * 68)
    print(f"[2] 기준({ref_name}) 대비 분기 시작 t*")
    print("=" * 68)
    print(f"  {'회차':14s} {'초기Δyaw':>9s} {'t*_yaw':>8s} {'동작@t*':>7s} "
          f"{'t*_pos':>8s} {'위치@t*_yaw':>14s} {'종말Δyaw':>9s}")
    results = []
    for r in R[1:]:
        dyaw      = r['yaw'] - ref['yaw']
        dyaw_detr = dyaw - dyaw[0]          # 초기 오프셋 제거 = '증가분'만
        dpos      = np.hypot(r['x'] - ref['x'], r['y'] - ref['y'])
        t_yaw = first_sustained(grid, dyaw_detr, YAW_THR)
        t_pos = first_sustained(grid, dpos,      POS_THR)

        if t_yaw is not None:
            i = idx_at(grid, t_yaw)
            loc  = f"({ref['x'][i]:+.1f},{ref['y'][i]:+.1f})"
            mlab = ref['lab'][i]
            ty   = f"{t_yaw:6.1f}s"
        else:
            loc, mlab, ty = "   -   ", "-", "   없음"
        tp = f"{t_pos:6.1f}s" if t_pos is not None else "   없음"

        print(f"  {r['name']:14s} {dyaw[0]:+8.1f}° {ty:>8s} {mlab:>7s} "
              f"{tp:>8s} {loc:>14s} {dyaw_detr[-1]:+8.1f}°")
        results.append(dict(r=r, dyaw_detr=dyaw_detr, dpos=dpos,
                            t_yaw=t_yaw, t_pos=t_pos))
    print()
    print("  판독:")
    print("   · 초기Δyaw 큼           → 초기화(정렬) 문제")
    print("   · 초기 0, t*에서 증가   → 그 구간이 원인 (동작@t* 이 방아쇠 후보)")
    print("   · 종말Δyaw 크면서 t* 늦음 → 후반 누적/붕괴형")

    # ---------------- [3] 그림 ----------------
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:
        print("\n(matplotlib 없음 → 그림 생략, 위 수치만으로 판독 가능)")
        return

    ref_segs = find_still_segments(grid, ref['lab'])

    def shade(ax):
        for (s, e, d) in ref_segs:
            ax.axvspan(s, e, color='0.85', zorder=0)

    fig, axes = plt.subplots(4, 1, figsize=(11, 13), sharex=True)

    ax = axes[0]
    for r in R:
        ax.plot(grid, np.hypot(r['x'], r['y']), lw=1, label=r['name'])
    shade(ax); ax.set_ylabel('dist_from_origin [m]')
    ax.legend(fontsize=7, ncol=4)
    ax.set_title('(a) distance from origin   (gray = still segments of ref)')

    ax = axes[1]
    for r in R:
        ax.plot(grid, r['yaw'], lw=1, label=r['name'])
    shade(ax); ax.set_ylabel('yaw (unwrap) [deg]')
    ax.set_title('(b) yaw')

    ax = axes[2]
    for res in results:
        ax.plot(grid, res['dyaw_detr'], lw=1, label=res['r']['name'])
        if res['t_yaw'] is not None:
            ax.axvline(res['t_yaw'], ls='--', lw=0.8)
    ax.axhline(YAW_THR, color='r', ls=':', lw=0.8)
    ax.axhline(-YAW_THR, color='r', ls=':', lw=0.8)
    shade(ax); ax.set_ylabel('d_yaw vs ref [deg]')
    ax.legend(fontsize=7, ncol=4)
    ax.set_title('(c) yaw diff vs ref (growth)   dashed = t*_yaw,  red = threshold')

    ax = axes[3]
    for res in results:
        ax.plot(grid, res['dpos'], lw=1, label=res['r']['name'])
        if res['t_pos'] is not None:
            ax.axvline(res['t_pos'], ls='--', lw=0.8)
    ax.axhline(POS_THR, color='r', ls=':', lw=0.8)
    shade(ax); ax.set_ylabel('d_pos vs ref [m]')
    ax.set_xlabel('t_sec [s]')
    ax.set_title('(d) position diff vs ref   dashed = t*_pos')

    fig.tight_layout()
    out = f"bifurcation_{ref_name}.png"
    fig.savefig(out, dpi=130)
    print(f"\n그림 저장 : {out}")


if __name__ == '__main__':
    main()
