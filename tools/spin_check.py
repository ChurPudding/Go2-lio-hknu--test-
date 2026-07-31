#!/usr/bin/env python3
"""
spin_check.py  --  제자리 회전 bag 으로 R_LB / LEVER 검증

인덱스 8번(위치·헤딩 검증)의 오프라인 판본.

원리
----
로봇이 제자리에서 연직축 둘레로 N 바퀴 돌면,
  - LiDAR 원점은 반지름 = LEVER 의 수평성분(0.322 m)인 원을 그린다
  - 회전축은 연직이어야 하고, 그 방향이 R_LB 에서 예상되는 값과 일치해야 한다
  - 연직축 둘레 누적 회전각이 로봇의 yaw 적분과 일치해야 한다

주의 — 순진한 yaw 추출은 틀린다
------------------------------
`2*atan2(qz,qw)` 는 roll·pitch 가 0 일 때만 맞다. L1 은 기울어져 장착돼 있어
camera_init(=시작 시점 LiDAR 프레임)의 z 축이 연직이 아니므로 과소평가된다.

회전축도 쿼터니언 벡터부를 그냥 평균하면 안 된다. θ 가 360° 를 지나면
sin(θ/2) 의 부호가 뒤집혀 방향이 반전되기 때문이다. 여기서는 **인접 프레임
사이의 상대회전**에서 축을 뽑아 누적한다(각속도 방향과 같아 부호 문제가 없다).

정지 시 up 방향
--------------
정지한 IMU 의 가속도계는 비력(specific force)을 재므로 **위쪽**을 가리킨다.
go2_calib.EXPECTED_REST_ACC = (+1.66, -1.90, -9.48) 이 곧 up 이다(부호 반전 금지).
진단정리의 클라우드 up (+0.154, -0.178, -0.972) 과 2도 이내로 일치한다.

사용
    python3 spin_check.py exp/spin_pointlio.csv [로봇_누적yaw_deg]
"""
import csv
import math
import sys

import numpy as np

UP = np.array([1.66, -1.90, -9.48])
UP = UP / np.linalg.norm(UP)
LEVER_H = 0.322          # LEVER 수평성분 [m]


def qmul(a, b):
    x1, y1, z1, w1 = a.T
    x2, y2, z2, w2 = b.T
    return np.stack([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2], 1)


def qconj(q):
    return np.stack([-q[:, 0], -q[:, 1], -q[:, 2], q[:, 3]], 1)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else 'exp/spin_pointlio.csv'
    ref_yaw = float(sys.argv[2]) if len(sys.argv) > 2 else None

    rows = list(csv.DictReader(open(src)))
    t = np.array([float(r['t']) for r in rows]); t -= t[0]
    P = np.array([[float(r['x']), float(r['y']), float(r['z'])] for r in rows])
    Q = np.array([[float(r['qx']), float(r['qy']), float(r['qz']), float(r['qw'])]
                  for r in rows])
    Q = Q / np.linalg.norm(Q, axis=1, keepdims=True)

    print('%d 점, %.1f 초, 평균 %.1f Hz' % (len(t), t[-1], len(t) / max(t[-1], 1e-9)))

    # ---- 인접 상대회전에서 축·각 뽑기 --------------------------------
    dq = qmul(qconj(Q[:-1]), Q[1:])
    dq = dq * np.sign(dq[:, 3:4])              # 항상 짧은 쪽 회전으로
    v = dq[:, :3]
    nv = np.linalg.norm(v, axis=1)
    dtheta = 2 * np.arctan2(nv, dq[:, 3])      # 각 스텝 회전량 [rad] (>=0)
    ok = nv > 1e-6
    axis = np.zeros_like(v)
    axis[ok] = v[ok] / nv[ok, None]

    # 회전량으로 가중해 평균 축 (많이 돈 구간이 신뢰도 높음)
    w = dtheta.copy()
    m = (w[:, None] * axis).sum(0)
    m = m / np.linalg.norm(m)

    print()
    print('--- 회전축')
    print('  데이터 추정   (%+.3f, %+.3f, %+.3f)' % tuple(m))
    print('  R_LB 기대 up  (%+.3f, %+.3f, %+.3f)' % tuple(UP))
    ang = math.degrees(math.acos(abs(np.clip(m @ UP, -1, 1))))
    print('  두 축의 각도  %.1f deg      <- 작을수록 R_LB 정확' % ang)

    # ---- 연직축 둘레 누적 회전 ----------------------------------------
    signed = dtheta * (axis @ UP)              # UP 성분만 투영해 누적
    total_up = np.degrees(signed.sum())
    total_all = np.degrees(dtheta.sum())
    print()
    print('--- 누적 회전')
    print('  연직축(UP) 둘레  %+.1f deg' % total_up)
    print('  총 회전량        %+.1f deg' % total_all)
    if ref_yaw is not None:
        print('  로봇 yaw 적분    %+.1f deg' % ref_yaw)
        print('  비율             %.3f' % (total_up / ref_yaw))

    # ---- LiDAR 원점 궤적: 연직축에 수직인 평면으로 투영 ----------------
    p = P - P[0]
    e1 = np.cross(UP, [1, 0, 0]);  e1 /= np.linalg.norm(e1)
    e2 = np.cross(UP, e1)
    u, vv = p @ e1, p @ e2

    A = np.stack([u, vv, np.ones_like(u)], 1)
    b = u ** 2 + vv ** 2
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0] / 2, c[1] / 2
    R = math.sqrt(max(c[2] + cx ** 2 + cy ** 2, 0))
    res = np.hypot(u - cx, vv - cy) - R

    print()
    print('--- LiDAR 원점 궤적 (연직축 수직 평면으로 투영)')
    print('  반지름     %.3f m      <- LEVER 수평성분 기대 %.3f m' % (R, LEVER_H))
    print('  잔차 RMS   %.3f m  최대 %.3f m' % (np.sqrt((res ** 2).mean()),
                                                np.abs(res).max()))
    print('  연직방향 변위  %+.3f ~ %+.3f m' % ((p @ UP).min(), (p @ UP).max()))

    # ---- 시간에 따른 각속도 (구간별로 이상한 데 없는지) ----------------
    print()
    print('--- 구간별 연직축 각속도 [deg/s]')
    dt = np.diff(t)
    rate = np.degrees(signed) / np.maximum(dt, 1e-9)
    for lo in range(0, int(t[-1]) + 1, 5):
        k = (t[1:] >= lo) & (t[1:] < lo + 5)
        if k.sum():
            print('  %4d~%3d s   평균 %+7.1f   최대 %+7.1f'
                  % (lo, lo + 5, rate[k].mean(), rate[k][np.argmax(np.abs(rate[k]))]))


if __name__ == '__main__':
    main()
