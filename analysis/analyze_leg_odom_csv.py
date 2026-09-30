#!/usr/bin/env python3
"""leg_odom_vs_robot.py 가 남긴 CSV 를 진단한다.

핵심: 축별 (내 속도 vs robot_odom 속도) 상관계수와 기울기.
  - 상관 높음 + 기울기≈1  → 양호 (재현 성공)
  - 상관 높음 + 기울기≠1  → 스케일 문제 (곱셈 하나로 고침)
  - 상관 높음 + 기울기<0  → 부호 뒤집힘 (규약 수정)
  - 상관 낮음             → 구조 문제 (프레임/접지/식 자체)

사용:  python3 analyze_leg_odom_csv.py leg_odom_cmp.csv
출력:  콘솔 요약표 + PNG 3장 (산점도 / 시계열 / 궤적)
"""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")            # 헤드리스에서도 저장되게
import matplotlib.pyplot as plt

VMIN = 0.05                       # 이동 판정 문턱 (m/s): 정지 샘플 제외


def verdict(r, slope, n):
    if n < 50:
        return "데이터 부족"
    if abs(r) < 0.5:
        return "구조 문제 (상관 낮음)"
    if slope < 0:
        return "부호 뒤집힘"
    if 0.85 <= slope <= 1.15:
        return "양호"
    return f"스케일 {slope:.2f} 배"


def analyze_axis(mine, odom):
    """이동 구간에서 상관·기울기·RMS 계산."""
    mask = np.abs(odom) > VMIN
    m, o = mine[mask], odom[mask]
    n = len(m)
    if n < 2:
        return dict(n=n, r=0.0, k=0.0, rms1=np.nan, rmsk=np.nan)
    r = np.corrcoef(m, o)[0, 1] if np.std(m) > 1e-9 else 0.0
    k = float(np.dot(m, o) / max(np.dot(m, m), 1e-12))   # o = k·m (원점통과)
    rms1 = float(np.sqrt(np.mean((o - m) ** 2)))          # k=1 잔차
    rmsk = float(np.sqrt(np.mean((o - k * m) ** 2)))      # k 적용 잔차
    return dict(n=n, r=float(r), k=k, rms1=rms1, rmsk=rmsk)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "leg_odom_cmp.csv"
    d = np.genfromtxt(path, delimiter=",", names=True)
    t = d["t"] - d["t"][0]
    axes = {"x": ("vmx", "vox"), "y": ("vmy", "voy"), "z": ("vmz", "voz")}

    print("\n" + "=" * 78)
    print(f"CSV 진단: {path}   (전체 {len(t)} 샘플, 이동 문턱 |v_odom|>{VMIN} m/s)")
    print("-" * 78)
    print(f"{'축':>3} {'n_move':>7} {'상관 r':>8} {'기울기 k':>9} "
          f"{'RMS(k=1)':>9} {'RMS(k_fit)':>11}   판정")
    print("-" * 78)
    res = {}
    for ax, (cm, co) in axes.items():
        s = analyze_axis(d[cm], d[co])
        res[ax] = s
        print(f"{ax:>3} {s['n']:>7} {s['r']:>8.3f} {s['k']:>9.3f} "
              f"{s['rms1']:>9.4f} {s['rmsk']:>11.4f}   {verdict(s['r'], s['k'], s['n'])}")
    print("=" * 78)
    print("읽는 법:")
    print("  상관 r 높고(>0.8) k≈1  → 재현 성공")
    print("  상관 r 높고 k≠1        → 스케일만 문제 (그 축에 k 곱하면 됨)")
    print("  상관 r 높고 k<0        → 부호 뒤집힘")
    print("  상관 r 낮음(<0.5)      → 구조 문제 (프레임/접지/식) — 스케일로 못 고침")

    # --- 그림 1: 축별 산점도 (내 vs odom) ---
    fig, axs = plt.subplots(1, 3, figsize=(13, 4.2))
    for i, (ax, (cm, co)) in enumerate(axes.items()):
        m, o = d[cm], d[co]
        axs[i].scatter(m, o, s=3, alpha=0.25)
        lim = np.nanpercentile(np.abs(np.concatenate([m, o])), 99)
        xs = np.linspace(-lim, lim, 10)
        axs[i].plot(xs, xs, "k--", lw=1, label="y=x (ideal)")
        axs[i].plot(xs, res[ax]["k"] * xs, "r-", lw=1.2,
                    label=f"fit k={res[ax]['k']:.2f}")
        axs[i].set(xlabel=f"mine v{ax}", ylabel=f"odom v{ax}",
                   title=f"{ax}-axis  r={res[ax]['r']:.2f}")
        axs[i].axhline(0, color="gray", lw=0.4); axs[i].axvline(0, color="gray", lw=0.4)
        axs[i].legend(fontsize=8); axs[i].set_aspect("equal", "box")
    fig.tight_layout(); fig.savefig("cmp_scatter.png", dpi=120)

    # --- 그림 2: 시계열 겹치기 (vx, vy) ---
    fig, axs = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    for i, ax in enumerate(("x", "y")):
        cm, co = axes[ax][0], axes[ax][1]
        axs[i].plot(t, d[cm], lw=0.8, label=f"mine v{ax}")
        axs[i].plot(t, d[co], lw=0.8, label=f"odom v{ax}", alpha=0.8)
        axs[i].set(ylabel=f"v{ax} (m/s)"); axs[i].legend(fontsize=8)
        axs[i].grid(alpha=0.3)
    axs[-1].set_xlabel("time (s)")
    fig.tight_layout(); fig.savefig("cmp_timeseries.png", dpi=120)

    # --- 그림 3: 궤적 겹치기 ---
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.plot(d["omx"], d["omy"], lw=1.2, label="robot_odom")
    ax.plot(d["pmx"], d["pmy"], lw=1.2, label="mine (integrated)", alpha=0.8)
    ax.set(xlabel="x (m)", ylabel="y (m)", title="trajectory")
    ax.legend(); ax.set_aspect("equal", "box"); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig("cmp_trajectory.png", dpi=120)

    print("\n저장됨: cmp_scatter.png / cmp_timeseries.png / cmp_trajectory.png")


if __name__ == "__main__":
    main()
