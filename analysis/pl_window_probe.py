#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pl_window_probe.py — 정지 구간의 yaw 드리프트를 회차별로 '숫자'로 뽑는다. (v2)

v1 대비 바뀐 점
  · 정지 판정을 '순간속도'가 아니라 '위치가 창 안에서 얼마나 안 벗어났나'로.
    L1 은 비반복 스캔이라 정지 중에도 정합 해가 실룩거려 LIO 속도가 0으로
    안 앉는다. 그래서 속도 기반 판정은 정지를 놓친다. 위치 이탈(반경) 기준은
    이 흔들림에 강하고, 재생 배속과도 무관하다.
        정지(t_i)  <=>  max_{|t_k - t_i|<T/2} |p_k - p_mean| < eps
        (eps=0.15 m, T=3 s)
  · 단, 이것도 여전히 LIO 위치 기반이다. '진짜 정지'는 --imu 로만 확정된다.

무엇을 하나
  1) 기준 회차에서 정지 시각창을 자동 검출 (위치 이탈 기준, bag 시간)
  2) 그 '같은 창'을 모든 회차에 적용해 yaw 드리프트율 w [deg/s] 를 잰다
        w = polyfit(t, yaw, 1) 의 기울기
  3) '가짜 이동속도'(정지인데 xy 로 움직였다고 보고한 양)도 함께 뽑는다
  4) --imu 주면 IMU 분산으로 각 창이 물리적 정지였는지 교차확인 (LIO 무관)

시각 기준
  traj_to_csv_v3.py 의 t_sec 은 벽시계(재생)시간. 0.5배속이면 bag = t_sec x 0.5.
  --rate 로 배속을 주면(기본 0.5) 모든 출력을 bag(실제) 시간 기준으로 환산.

사용
  python3 pl_window_probe.py <기준.csv> [비교.csv ...] [--rate 0.5]
  python3 pl_window_probe.py ref.csv a.csv --win 199:220 --win 415:436
  python3 pl_window_probe.py ref.csv a.csv --imu imu.csv
     (imu.csv 컬럼: t_sec, ax, ay, az, gx, gy, gz)
"""

import argparse
import os
import numpy as np

# ------------------------- 기본 임계값 -------------------------
EPS       = 0.15    # 정지 판정: 이 반경(m) 안에 머물면 정지
PROBE_WIN = 3.0     # 정지 판정용 창 폭(bag s)
MIN_STILL = 5.0     # 이 시간(bag s) 이상 지속돼야 정지 구간으로 인정
W_STABLE  = 1.0     # |w| < 이 값 → 안정 [deg/s]
W_COLLAPSE = 5.0    # |w| > 이 값 → 붕괴 [deg/s]
IMU_ACC_VAR = 0.02  # IMU 정지 판정 (m/s^2)^2
IMU_GYR_VAR = 0.001 # IMU 정지 판정 (rad/s)^2
# -------------------------------------------------------------


def load(path, rate):
    """CSV → bag시간 t, x, y, yaw_deg(감김해제)."""
    a = np.genfromtxt(path, delimiter=',', skip_header=1)
    if a.ndim != 2 or a.shape[1] < 9:
        raise ValueError(f"{path}: 컬럼이 예상과 다름 (shape={a.shape})")
    t   = a[:, 0] * rate                       # 벽시계 → bag 시간
    yaw = np.rad2deg(np.unwrap(np.deg2rad(a[:, 8])))
    return dict(name=os.path.splitext(os.path.basename(path))[0],
                t=t, x=a[:, 1], y=a[:, 2], yaw=yaw)


def find_still_windows(d, eps=EPS, min_dur=MIN_STILL):
    """정지 시각창 [(t0,t1), ...] (bag 시간).
    구간 성장: 후보 구간의 모든 점이 그 구간 중심에서 반경 eps 안이면
    계속 이어붙인다. 벗어나면 구간을 닫는다. (양끝을 깎지 않음)
    보행 중이면 금방 eps 를 벗어나 짧게 닫히므로 min_dur 로 걸러진다.
    """
    t, x, y = d['t'], d['x'], d['y']
    n = len(t)
    wins = []
    i = 0
    while i < n:
        j = i + 1
        while j < n:
            xs = x[i:j + 1]; ys = y[i:j + 1]
            spread = np.hypot(xs - xs.mean(), ys - ys.mean()).max()
            if spread < eps:
                j += 1
            else:
                break
        if t[j - 1] - t[i] >= min_dur:
            wins.append((t[i], t[j - 1]))
        i = j if j > i + 1 else i + 1
    return wins


def drift_in_window(d, w0, w1):
    m = (d['t'] >= w0) & (d['t'] <= w1)
    if m.sum() < 3:
        return None
    t = d['t'][m]; x = d['x'][m]; y = d['y'][m]; yaw = d['yaw'][m]
    dur = t[-1] - t[0]
    coef = np.polyfit(t, yaw, 1)
    w = float(coef[0])
    res = float((yaw - np.polyval(coef, t)).std())
    path = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
    return dict(n=int(m.sum()), dur=dur, w=w, res=res,
                yaw_change=float(yaw[-1] - yaw[0]),
                fake_v=(path / dur if dur > 0 else 0.0),
                cx=float(x.mean()), cy=float(y.mean()))


def verdict(w):
    aw = abs(w)
    if aw < W_STABLE:
        return "안정"
    if aw > W_COLLAPSE:
        return "붕괴"
    return "부분"


def imu_still_check(imu_path, rate, wins):
    a = np.genfromtxt(imu_path, delimiter=',', skip_header=1)
    if a.ndim != 2 or a.shape[1] < 7:
        print(f"  (IMU CSV 컬럼 부족: {a.shape} — 교차확인 생략)")
        return
    t = a[:, 0] * rate
    acc = a[:, 1:4]; gyr = a[:, 4:7]
    print("\n" + "=" * 60)
    print("[IMU 교차확인] 각 창이 물리적으로 정지였나 (LIO 무관)")
    print("=" * 60)
    print(f"  {'창(bag s)':>15s} {'σ²_acc':>10s} {'σ²_gyr':>10s}  판정")
    for (w0, w1) in wins:
        m = (t >= w0) & (t <= w1)
        if m.sum() < 3:
            print(f"  {w0:6.1f}~{w1:6.1f}   (IMU 표본 부족)")
            continue
        va = float(np.mean(np.var(acc[m], axis=0)))
        vg = float(np.mean(np.var(gyr[m], axis=0)))
        ok = (va < IMU_ACC_VAR) and (vg < IMU_GYR_VAR)
        tag = "물리적 정지 확인" if ok else "움직임 있음(정지 아님)"
        print(f"  {w0:6.1f}~{w1:6.1f}   {va:10.4f} {vg:10.5f}  {tag}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+")
    ap.add_argument("--rate", type=float, default=0.5)
    ap.add_argument("--win", action="append", default=[],
                    help="창 직접 지정 'bag_t0:bag_t1' (여러 번 가능)")
    ap.add_argument("--eps", type=float, default=EPS,
                    help="정지 판정 반경 [m] (기본 0.15)")
    ap.add_argument("--probe-win", type=float, default=PROBE_WIN)
    ap.add_argument("--min-still", type=float, default=MIN_STILL)
    ap.add_argument("--imu", default=None)
    args = ap.parse_args()

    runs = [load(p, args.rate) for p in args.csvs]
    ref = runs[0]
    print(f"배속 rate={args.rate}  →  시각·속도·드리프트율은 bag(실제) 시간 기준")
    print(f"기준 회차: {ref['name']}   "
          f"(정지판정: 반경 {args.eps} m 안에 {args.min_still}s 이상 머묾)\n")

    if args.win:
        wins = [tuple(float(v) for v in s.split(":")) for s in args.win]
        print("사용 창(직접 지정, bag 시간):")
    else:
        wins = find_still_windows(ref, args.eps, args.min_still)
        print("사용 창(기준 회차 자동 검출, bag 시간):")
    for (w0, w1) in wins:
        print(f"   {w0:7.1f} ~ {w1:7.1f} s  ({w1-w0:.1f}s)")
    if not wins:
        print("   (정지 구간 없음 — --eps 를 키우거나 --win 으로 직접 지정)")
        return

    for (w0, w1) in wins:
        rc = drift_in_window(ref, w0, w1)
        cxy = f"({rc['cx']:+.1f},{rc['cy']:+.1f})" if rc else "(?)"
        print("\n" + "=" * 76)
        print(f"[정지 창 {w0:.1f}~{w1:.1f}s ({w1-w0:.1f}s)]  기준 중심위치 {cxy}")
        print("=" * 76)
        print(f"  {'회차':16s} {'표본':>4s} {'yaw변화':>9s} "
              f"{'드리프트w':>10s} {'잔차RMS':>8s} {'가짜속도':>10s}  판정")
        for d in runs:
            r = drift_in_window(d, w0, w1)
            if r is None:
                print(f"  {d['name']:16s}  (이 창에 표본 부족)")
                continue
            print(f"  {d['name']:16s} {r['n']:4d} {r['yaw_change']:+8.1f}° "
                  f"{r['w']:+8.2f}°/s {r['res']:7.2f}° {r['fake_v']*100:8.1f}cm/s"
                  f"  {verdict(r['w'])}")
        print("  판독: w≈0 안정 / 수°/s 붕괴.  가짜속도 크면 그 회차 LIO가 정지를 정지로 못 봄.")

    if args.imu:
        imu_still_check(args.imu, args.rate, wins)


if __name__ == "__main__":
    main()
