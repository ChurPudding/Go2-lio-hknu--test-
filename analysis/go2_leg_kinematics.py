#!/usr/bin/env python3
"""Go2 다리 기구학 순수 코어 (ROS 의존성 없음).

URDF(unitreerobotics/unitree_ros, go2_description) 실측치 기반.
- 순기구학 fk_foot: 관절각 -> 몸통(base)좌표계 발끝 위치
- 수치 자코비안 jacobian: 발속도 = J @ dq (해석적 유도 대신 중앙차분, 실수 최소화)
- body_velocity: v_b = -(1/|S|) Σ (J·dq + ω×p)   ← 다리 오도메트리 심장

축 규약(URDF 확인): hip=x축 회전, thigh·calf=y축 회전.
부호 규약: 양의 thigh 각이 발을 뒤(-x)로 보냄 (URDF 기준).
스케일 상수는 일부러 안 넣음 — robot_odom과 피팅해서 찾을 값이므로.
"""

import numpy as np

# ---- URDF 실측 상수 ----
L1 = 0.213      # 허벅지 길이 (m)
L2 = 0.213      # 종아리 길이 (m)
D_H = 0.0955    # 고관절 좌우 오프셋 hip->thigh (m)

# 몸통중심(base) -> 고관절 장착 위치 (m)
HIP_XYZ = {
    "FL": np.array([ 0.1934,  0.0465, 0.0]),
    "FR": np.array([ 0.1934, -0.0465, 0.0]),
    "RL": np.array([-0.1934,  0.0465, 0.0]),
    "RR": np.array([-0.1934, -0.0465, 0.0]),
}
# thigh y-오프셋 부호 (좌 +, 우 -)
SIDE = {"FL": +1.0, "FR": -1.0, "RL": +1.0, "RR": -1.0}

# Unitree 네이티브 motor_state 인덱스 (URDF 순서 아님)
MOTOR_IDX = {
    "FR": (0, 1, 2), "FL": (3, 4, 5),
    "RR": (6, 7, 8), "RL": (9, 10, 11),
}
LEGS = ("FL", "FR", "RL", "RR")


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def fk_foot(leg, q1, q2, q3):
    """관절각(rad) -> 몸통(base)좌표계 발끝 위치 (m)."""
    s = SIDE[leg]
    v_calf = Ry(q3) @ np.array([0.0, 0.0, -L2])       # 종아리 끝 (calf 프레임)
    v_thigh = np.array([0.0, 0.0, -L1]) + v_calf       # 허벅지 원점 기준
    v_leg = Ry(q2) @ v_thigh                           # thigh 회전 적용
    v = np.array([0.0, s * D_H, 0.0]) + v_leg          # 고관절 좌우 오프셋
    p = Rx(q1) @ v                                      # hip roll 적용
    return HIP_XYZ[leg] + p                             # 고관절 장착 위치 더함


def jacobian(leg, q, eps=1e-6):
    """중앙차분 3x3 자코비안: v_foot_body = J @ dq."""
    q = np.asarray(q, dtype=float)
    J = np.zeros((3, 3))
    for i in range(3):
        d = np.zeros(3); d[i] = eps
        J[:, i] = (fk_foot(leg, *(q + d)) - fk_foot(leg, *(q - d))) / (2 * eps)
    return J


def stance_by_height(q_all, margin=0.02):
    """발 높이(z) 기반 접지 판정: 가장 낮은 발에서 margin 이내인 다리들.

    tau_est 기반이 준비되기 전의 기본값. margin 이 첫 튜닝 파라미터.
    """
    z = {leg: fk_foot(leg, *q_all[leg])[2] for leg in LEGS}
    z_min = min(z.values())
    return [leg for leg in LEGS if z[leg] <= z_min + margin]


def stance_by_force(q_all, tau_all, thr=20.0):
    """tau 기반 접지 판정: 발 수직힘 |Fz| = |(J^-T τ)_z| > thr 인 다리.

    스윙 다리는 관절토크가 작아 |Fz| 가 작다 → 자연히 제외된다.
    thr(N) 은 첫 튜닝 파라미터. Go2 다리당 지지력 대략 30~70N.
    """
    stance = []
    for leg in LEGS:
        f = foot_force(leg, q_all[leg], tau_all[leg])
        if abs(f[2]) > thr:
            stance.append(leg)
    return stance if stance else list(LEGS)   # 다 걸러지면 안전하게 전체


def foot_force(leg, q, tau):
    """관절토크 -> 발끝 힘 (몸통좌표계): J^T f = τ  →  f = J^-T τ."""
    J = jacobian(leg, q)
    return np.linalg.solve(J.T, np.asarray(tau, dtype=float))


def body_velocity(q_all, dq_all, omega, stance, use_gyro=True,
                  reject_outliers=False, k_reject=2.0):
    """몸통좌표계 속도 v_b = -(1/|S|) Σ (J·dq + ω×p).

    reject_outliers: 지지 다리 중 추정치가 중앙값에서 크게 벗어난(스윙 오분류)
                     다리를 버리고 평균 → 접지 판정 실수에 강건.
    """
    omega = np.asarray(omega, dtype=float)
    contrib = []
    for leg in stance:
        p = fk_foot(leg, *q_all[leg])
        J = jacobian(leg, q_all[leg])
        v_foot_body = J @ np.asarray(dq_all[leg], dtype=float)
        term = v_foot_body + (np.cross(omega, p) if use_gyro else 0.0)
        contrib.append(-term)
    if not contrib:
        return np.zeros(3)
    contrib = np.array(contrib)
    if reject_outliers and len(contrib) >= 3:
        med = np.median(contrib, axis=0)
        dist = np.linalg.norm(contrib - med, axis=1)
        mad = np.median(dist) + 1e-6
        keep = contrib[dist <= k_reject * mad]
        if len(keep) > 0:
            contrib = keep
    return np.mean(contrib, axis=0)


def unpack_motor(q_vec, dq_vec):
    """motor_state 12벡터 -> 다리별 (q1,q2,q3),(dq1,dq2,dq3) dict."""
    q_all, dq_all = {}, {}
    for leg, (i, j, k) in MOTOR_IDX.items():
        q_all[leg] = (q_vec[i], q_vec[j], q_vec[k])
        dq_all[leg] = (dq_vec[i], dq_vec[j], dq_vec[k])
    return q_all, dq_all


if __name__ == "__main__":
    # 자체 점검 (로봇 없이)
    print("=== FK 점검: 명목 스탠드 자세 (q1=0, q2=0.9, q3=-1.8) ===")
    for leg in LEGS:
        p = fk_foot(leg, 0.0, 0.9, -1.8)
        print(f"  {leg}: foot = [{p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f}] m")

    print("\n=== 자코비안 vs 접지속도 관계 점검 ===")
    q = (0.0, 0.9, -1.8)
    J = jacobian("FL", q)
    print("  det(J) =", f"{np.linalg.det(J):+.5f}", "(0 아니면 비특이 = 정상)")

    print("\n=== 특이점 자유 확인: calf 한계 전 구간에서 det(J)≠0 ? ===")
    dets = [np.linalg.det(jacobian("FL", (0.0, 0.5, q3)))
            for q3 in np.linspace(-2.7227, -0.83776, 15)]
    print("  |det| 최소 =", f"{min(abs(d) for d in dets):.5f}",
          "→ 0이 아니면 관절한계 안에서 특이점 없음")

    print("\n=== body_velocity 점검: 접지발을 뒤로 미는 보행 ===")
    q_all = {leg: (0.0, 0.9, -1.8) for leg in LEGS}
    # thigh +방향 → 접지발이 몸 기준 뒤로 밀림 → 몸은 앞으로(+x) 나감
    dq_all = {leg: (0.0, +0.5, 0.0) for leg in LEGS}
    omega = (0.0, 0.0, 0.0)
    stance = stance_by_height(q_all)
    v = body_velocity(q_all, dq_all, omega, stance)
    print(f"  접지 다리 = {stance}")
    print(f"  v_body = [{v[0]:+.4f} {v[1]:+.4f} {v[2]:+.4f}] m/s")
    print("  (접지발을 뒤로 밀면 x>0 = 몸이 앞으로 → 부호 규약 일관)")
