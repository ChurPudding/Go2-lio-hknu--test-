#!/usr/bin/env python3
"""force_thr · alpha 오프라인 스윕 (record_raw_leg.py 의 원시 CSV 사용).

핵심 최적화: 기구학(FK·자코비안·발힘·다리별 속도기여)은 force_thr/alpha 와
무관하므로 '한 번만' 계산해 캐시한다. 이후 각 파라미터 조합은 접지 선택+평균+
EMA 만 다시 하므로 재생 없이 수십 조합을 순식간에 평가한다.

  python3 sweep_leg_odom.py leg_raw.csv
"""

import sys
import numpy as np
import go2_leg_kinematics as kin

THR_LIST = [10, 15, 20, 25, 30, 40, 50]      # force_thr 후보 (N)
ALPHA_LIST = [1.0, 0.3, 0.1, 0.05]           # 저역통과 (1.0=끔)
VMIN = 0.05                                   # 이동 판정 문턱


def load_raw(path):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    t = d[:, 0]
    q = d[:, 1:13]; dq = d[:, 13:25]; tau = d[:, 25:37]
    w = d[:, 37:40]
    vo = d[:, 40:43]
    wo = d[:, 45:48] if d.shape[1] >= 48 else None   # odom 각속도(신형 CSV만)
    return t, q, dq, tau, w, vo, wo


def precompute(q, dq, tau, w):
    """행마다·다리마다: 속도기여(gyro on/off) 와 발수직힘 |Fz| 를 미리 계산."""
    n = len(q)
    legs = kin.LEGS
    contrib_g = np.zeros((n, 4, 3))    # -(J dq + w×p)
    contrib_n = np.zeros((n, 4, 3))    # -(J dq)
    fz = np.zeros((n, 4))              # |foot force z|
    for r in range(n):
        q_all, dq_all = kin.unpack_motor(q[r], dq[r])
        _, tau_all = kin.unpack_motor(tau[r], tau[r])
        om = w[r]
        for li, leg in enumerate(legs):
            p = kin.fk_foot(leg, *q_all[leg])
            J = kin.jacobian(leg, q_all[leg])
            vfb = J @ np.asarray(dq_all[leg])
            contrib_n[r, li] = -vfb
            contrib_g[r, li] = -(vfb + np.cross(om, p))
            f = np.linalg.solve(J.T, np.asarray(tau_all[leg]))
            fz[r, li] = abs(f[2])
        if r % 20000 == 0:
            print(f"  precompute {r}/{n}")
    return contrib_g, contrib_n, fz


def eval_config(contrib, fz, thr, alpha, reject=True):
    """접지=|Fz|>thr, 평균(+이상치제거), EMA(alpha) → v_mine 시퀀스 (n,3)."""
    n = contrib.shape[0]
    v = np.zeros((n, 3))
    for r in range(n):
        sel = fz[r] > thr
        c = contrib[r][sel]
        if len(c) == 0:
            c = contrib[r]                       # 다 걸러지면 전체
        if reject and len(c) >= 3:
            med = np.median(c, axis=0)
            dist = np.linalg.norm(c - med, axis=1)
            mad = np.median(dist) + 1e-6
            k = c[dist <= 2.0 * mad]
            if len(k):
                c = k
        v[r] = c.mean(axis=0)
    if alpha < 1.0:                              # EMA 저역통과
        out = np.zeros_like(v); out[0] = v[0]
        for r in range(1, n):
            out[r] = alpha * v[r] + (1 - alpha) * out[r - 1]
        v = out
    return v


def corr_k(mine, odom):
    mask = np.abs(odom) > VMIN
    m, o = mine[mask], odom[mask]
    if len(m) < 2 or np.std(m) < 1e-9:
        return 0.0, 0.0, len(m)
    r = np.corrcoef(m, o)[0, 1]
    k = np.dot(m, o) / max(np.dot(m, m), 1e-12)
    return float(r), float(k), len(m)


def ema(x, alpha):
    if alpha >= 1.0:
        return x
    out = np.zeros_like(x); out[0] = x[0]
    for i in range(1, len(x)):
        out[i] = alpha * x[i] + (1 - alpha) * out[i - 1]
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "leg_raw.csv"
    t, q, dq, tau, w, vo, wo = load_raw(path)
    print(f"원시 {len(t)} 행 로드. 기구학 사전계산 중(1회)...")
    cg, cn, fz = precompute(q, dq, tau, w)

    print("\n force_thr 스윕 (gyro on, reject on)")
    print(f"{'thr':>5} {'alpha':>6} {'접지평균':>7} {'r_x':>7} {'k_x':>7} "
          f"{'r_y':>7} {'k_y':>7}")
    print("-" * 52)
    best = None
    for thr in THR_LIST:
        mean_stance = np.mean(np.sum(fz > thr, axis=1))
        for alpha in ALPHA_LIST:
            v = eval_config(cg, fz, thr, alpha)
            rx, kx, _ = corr_k(v[:, 0], vo[:, 0])
            ry, ky, _ = corr_k(v[:, 1], vo[:, 1])
            print(f"{thr:>5} {alpha:>6} {mean_stance:>7.2f} "
                  f"{rx:>7.3f} {kx:>7.3f} {ry:>7.3f} {ky:>7.3f}")
            if best is None or rx > best[0]:
                best = (rx, kx, thr, alpha)
        print("-" * 52)
    print(f"\n최적 (x축 r 최대): force_thr={best[2]}  alpha={best[3]}  "
          f"→ r_x={best[0]:.3f}  k_x={best[1]:.3f}")
    print("이 조합을 leg_odom_vs_robot.py 파라미터로 넣으면 실주행에 그대로 적용됩니다:")
    print(f"  --ros-args -p stance_mode:=force -p force_thr:={best[2]} -p alpha:={best[3]}")

    # --- yaw 비교: 내 자이로 wz vs odom 각속도 z (접지·스케일 무관) ---
    if wo is not None:
        print("\n yaw 비교 (내 자이로 ωz  vs  robot_odom 각속도 z)")
        print(f"{'alpha':>6} {'r_yaw':>8} {'k_yaw':>8}")
        print("-" * 24)
        for alpha in (1.0, 0.3, 0.1):
            r, k, _ = corr_k(ema(w[:, 2], alpha), wo[:, 2])
            print(f"{alpha:>6} {r:>8.3f} {k:>8.3f}")
        print("  r_yaw≈1, k_yaw≈1 이면 → robot_odom 은 yaw 를 자이로 그대로 씀 (2D 완성)")
    else:
        print("\n[yaw] 이 CSV엔 odom 각속도가 없습니다. 새 record_raw_leg.py 로 다시 기록하세요.")


if __name__ == "__main__":
    main()
