#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zupt_offline.py — 궤적 CSV 에 yaw hold(ZUPT) 를 오프라인으로 적용하고,
                  보정 전/후 드리프트를 비교한다. (로봇·ROS 불필요)

목적
  실시간 zupt_filter_yaw.py 와 '수학적으로 동일한' yaw hold 를 CSV 에 적용해,
  정지창의 yaw 붕괴가 사라지는지 로봇 없이 먼저 확인한다.

yaw hold (B 방식) — 실시간 노드와 동일
  정지창 안: yaw 를 '정지 진입 시점 값'으로 고정
  정지창 밖: 입력 그대로
      ψ_out(t) = ψ_hold           (정지창 안)
                 ψ_in(t)          (정지창 밖)
  * 위치(xy)도 같은 방식으로 진입 시점 값에 고정(원본 ZUPT 와 동일).
  * B 방식이므로 정지 해제 순간 불연속(툭 튐) 이 생길 수 있다(예상된 한계).

정지창 결정 (우선순위)
  1) --win t0:t1  직접 지정 (bag 시간). gyro 로 확인된 진짜 정지 권장.
  2) 없으면 자동 검출 (위치가 반경 eps 안에 min_dur 이상 머문 구간).

시각
  CSV t_sec 은 벽시계(0.5배속). --rate 로 bag 시간 환산(기본 0.5).
  --win 은 bag 시간으로 준다.

사용
  python3 zupt_offline.py run_PL_run1_d.csv --win 209.8:246
  python3 zupt_offline.py run_PL_run1_d.csv            # 자동 검출
  # 여러 회차 한 번에 비교
  python3 zupt_offline.py run_PL_run1_a.csv run_PL_run1_d.csv --win 209.8:246

출력
  · 각 정지창의 보정 전/후 yaw 드리프트 w [deg/s]
  · 보정된 CSV 저장: <원본>_zupt.csv  (yaw_deg, qx..qw 갱신)
"""

import argparse
import os
import math
import numpy as np

EPS = 0.15        # 자동 검출: 정지 반경 [m]
MIN_DUR = 5.0     # 자동 검출: 최소 지속 [s]


def load(path, rate):
    a = np.genfromtxt(path, delimiter=',', skip_header=1)
    if a.ndim != 2 or a.shape[1] < 10:
        raise ValueError(f"{path}: 컬럼 예상과 다름 (shape={a.shape})")
    return a, a[:, 0] * rate      # 원배열, bag시간


def yaw_unwrap(yaw_deg):
    return np.rad2deg(np.unwrap(np.deg2rad(yaw_deg)))


def find_windows(t, x, y, eps=EPS, min_dur=MIN_DUR):
    """구간 성장 방식 정지 검출 (window_probe 와 동일)."""
    n = len(t); wins = []; i = 0
    while i < n:
        j = i + 1
        while j < n:
            xs = x[i:j + 1]; ys = y[i:j + 1]
            if np.hypot(xs - xs.mean(), ys - ys.mean()).max() < eps:
                j += 1
            else:
                break
        if t[j - 1] - t[i] >= min_dur:
            wins.append((t[i], t[j - 1]))
        i = j if j > i + 1 else i + 1
    return wins


def yaw_to_quat_z(yaw_deg):
    """평면 yaw → quaternion (z축 회전만). roll/pitch 는 원본 유지가 원칙이나
    오프라인에선 yaw 고정 효과 확인이 목적이므로 z회전으로 재구성."""
    r = np.deg2rad(yaw_deg) / 2.0
    return 0.0, 0.0, math.sin(r), math.cos(r)


def drift_rate(t, yaw_u):
    """정지창 안 yaw 를 시간에 직선맞춤한 기울기 [deg/s] 와 잔차RMS."""
    if len(t) < 3:
        return float('nan'), float('nan'), float('nan')
    coef = np.polyfit(t, yaw_u, 1)
    res = float((yaw_u - np.polyval(coef, t)).std())
    return float(coef[0]), res, float(yaw_u[-1] - yaw_u[0])


def apply_hold(a, tb, wins):
    """정지창마다 위치·yaw 를 진입 시점 값으로 고정. 보정된 배열 반환."""
    out = a.copy()
    yaw_u = yaw_unwrap(a[:, 8])
    out_yaw = yaw_u.copy()
    for (w0, w1) in wins:
        idx = np.where((tb >= w0) & (tb <= w1))[0]
        if len(idx) < 2:
            continue
        k0 = idx[0]
        # 진입 시점 값으로 고정 (B 방식)
        out[idx, 1] = a[k0, 1]              # x
        out[idx, 2] = a[k0, 2]              # y
        out[idx, 3] = a[k0, 3]              # z
        out_yaw[idx] = yaw_u[k0]            # yaw(unwrap)
    # yaw 를 ±180 으로 되감고, quaternion 재구성, dist 갱신
    out[:, 8] = (out_yaw + 180) % 360 - 180
    for i in range(len(out)):
        qx, qy, qz, qw = yaw_to_quat_z(out[i, 8])
        out[i, 4], out[i, 5], out[i, 6], out[i, 7] = qx, qy, qz, qw
    out[:, 9] = np.hypot(out[:, 1], out[:, 2])
    return out, out_yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+")
    ap.add_argument("--rate", type=float, default=0.5)
    ap.add_argument("--win", action="append", default=[],
                    help="정지창 'bag_t0:bag_t1' (여러 번 가능). 없으면 자동검출")
    ap.add_argument("--eps", type=float, default=EPS)
    ap.add_argument("--min-dur", type=float, default=MIN_DUR)
    args = ap.parse_args()

    print(f"배속 rate={args.rate} → bag 시간 기준\n")

    for path in args.csvs:
        name = os.path.splitext(os.path.basename(path))[0]
        a, tb = load(path, args.rate)
        x, y = a[:, 1], a[:, 2]

        if args.win:
            wins = [tuple(float(v) for v in s.split(":")) for s in args.win]
            how = "직접 지정"
        else:
            wins = find_windows(tb, x, y, args.eps, args.min_dur)
            how = "자동 검출"

        print("=" * 70)
        print(f"[{name}]  정지창 {how} ({len(wins)}개)")
        print("=" * 70)

        yaw_before = yaw_unwrap(a[:, 8])
        out, yaw_after = apply_hold(a, tb, wins)

        print(f"  {'정지창(bag s)':>16s} {'지속':>6s} "
              f"{'w_before':>10s} {'w_after':>9s} {'개선':>8s}")
        for (w0, w1) in wins:
            idx = np.where((tb >= w0) & (tb <= w1))[0]
            if len(idx) < 3:
                print(f"  {w0:7.1f}~{w1:6.1f}  (표본부족)")
                continue
            wb, rb, _ = drift_rate(tb[idx], yaw_before[idx])
            wa, ra, _ = drift_rate(tb[idx], yaw_after[idx])
            imp = "→0" if abs(wa) < 0.5 else f"{100*(1-abs(wa)/max(abs(wb),1e-9)):.0f}%"
            print(f"  {w0:7.1f}~{w1:6.1f} {w1-w0:5.1f}s "
                  f"{wb:+8.2f}°/s {wa:+7.2f}°/s {imp:>8s}")

        # 보정 CSV 저장
        outpath = os.path.join(os.path.dirname(path) or ".", name + "_zupt.csv")
        hdr = "t_sec,x,y,z,qx,qy,qz,qw,yaw_deg,dist_from_origin"
        np.savetxt(outpath, out, delimiter=",", header=hdr, comments="",
                   fmt="%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f")
        print(f"  보정 CSV 저장: {outpath}")
        print(f"  → 검증: python3 pl_window_probe.py {name}_zupt.csv "
              f"--win {'  '.join(f'{int(w0)}:{int(w1)}' for w0,w1 in wins)}\n")


if __name__ == "__main__":
    main()
