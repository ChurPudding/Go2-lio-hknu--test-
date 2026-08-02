#!/usr/bin/env python3
"""
imu_deadreckon.py  --  IMU 만으로 추측항법을 돌려 LIO 궤적과 비교

목적
----
실외에서 LIO 의 수평 위치가 사실상 **IMU 이중적분 그대로인지** 확인한다.

  - 두 궤적이 비슷하다  -> LiDAR 가 수평에 기여한 것이 없다. 관측성 부족 확정
  - LIO 가 훨씬 낫다     -> LiDAR 가 일부 잡아줬다. 다른 원인을 봐야 함

방법
----
`l1_imu_fix.py` 와 같은 신호를 쓴다.
  - 자이로: `/utlidar/imu` (L1, LiDAR 프레임과 평행)
  - 가속도: `/lowstate` 본체를 `R_LB` 로 LiDAR 프레임으로 회전, ACC_SCALE 보정

초기 정지 구간(기본 5초)에서 자이로 바이어스와 중력 방향을 추정한 뒤,
자세를 자이로로 적분하고 가속도에서 중력을 빼 위치를 이중적분한다.

**LiDAR 를 전혀 쓰지 않는다.** 그래서 이 결과는 "보정이 하나도 없을 때"의 하한이다.

사용
    python3 imu_deadreckon.py <bag_0.db3> [출력.csv] [정지초]
"""
import csv
import math
import struct
import sqlite3
import sys

import numpy as np

R_LB = np.array([
    [+0.523029, -0.838576, +0.152420],
    [-0.810712, -0.544668, -0.214668],
    [+0.263034, -0.011292, -0.964721],
])
ACC_SCALE = 9.807 / 9.465
G = 9.807

OFF_ACC_LOWSTATE = None      # LowState 는 필드가 많아 오프셋 계산이 복잡 -> 아래 참조


def read_imu(con):
    """/utlidar/imu 에서 자이로만 뽑는다 (sensor_msgs/Imu)."""
    r = con.execute("SELECT id FROM topics WHERE name='/utlidar/imu'").fetchone()
    if r is None:
        sys.exit('/utlidar/imu 없음')
    out = []
    for ts, b in con.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (r[0],)):
        E = '<' if b[1] == 1 else '>'
        p = [4]

        def al(n):
            o = (p[0] - 4) % n
            if o:
                p[0] += n - o

        def u32():
            al(4); v = struct.unpack_from(E + 'I', b, p[0])[0]; p[0] += 4; return v

        def s():
            n = u32(); v = b[p[0]:p[0] + n - 1]; p[0] += n; return v

        u32(); u32(); s()                       # stamp, frame_id
        al(8); p[0] += 8 * 4                    # orientation (4 double)
        p[0] += 8 * 9                           # orientation_covariance
        gyro = struct.unpack_from(E + '3d', b, p[0]); p[0] += 24
        out.append((ts * 1e-9, *gyro))
    return np.array(out)


def read_lowstate_acc(con):
    """/lowstate 의 imu_state.accelerometer (float32[3]) 를 뽑는다.

    unitree_go/msg/LowState 배치(CDR, 헤더 4바이트 이후 기준):
      head[2] uint8            0
      level_flag uint8         2
      frame_reserve uint8      3
      sn[2] uint32             4
      version[2] uint32       12
      bandwidth uint16        20
      -- IMUState (align 4) --  24
        quaternion[4] f32      24
        gyroscope[3] f32       40
        accelerometer[3] f32   52
        rpy[3] f32             64
        temperature int8       76
    """
    r = con.execute("SELECT id FROM topics WHERE name='/lowstate'").fetchone()
    if r is None:
        sys.exit('/lowstate 없음')
    out = []
    for ts, b in con.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (r[0],)):
        if len(b) < 4 + 80:
            continue
        E = '<' if b[1] == 1 else '>'
        a = struct.unpack_from(E + '3f', b, 4 + 52)
        out.append((ts * 1e-9, *a))
    return np.array(out)


def main():
    db = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else 'imu_deadreckon.csv'
    still = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

    con = sqlite3.connect(db)
    print('자이로 읽는 중...')
    Gy = read_imu(con)
    print('  %d 개, %.1f 초 (%.0f Hz)' % (len(Gy), Gy[-1, 0] - Gy[0, 0],
                                          len(Gy) / (Gy[-1, 0] - Gy[0, 0])))
    print('가속도 읽는 중...')
    Ac = read_lowstate_acc(con)
    print('  %d 개, %.1f 초 (%.0f Hz)' % (len(Ac), Ac[-1, 0] - Ac[0, 0],
                                          len(Ac) / (Ac[-1, 0] - Ac[0, 0])))

    t0 = max(Gy[0, 0], Ac[0, 0])
    t1 = min(Gy[-1, 0], Ac[-1, 0])

    # 자이로 시각 기준으로 통일
    m = (Gy[:, 0] >= t0) & (Gy[:, 0] <= t1)
    t = Gy[m, 0]
    w = Gy[m, 1:4]
    acc_body = np.stack([np.interp(t, Ac[:, 0], Ac[:, k]) for k in (1, 2, 3)], 1)
    acc = (R_LB @ (acc_body * ACC_SCALE).T).T       # LiDAR 프레임

    # --- 초기 정지 구간에서 바이어스·중력 추정 -------------------------
    k = t - t[0] < still
    if k.sum() < 10:
        sys.exit('정지 구간이 너무 짧다')
    bias_w = w[k].mean(0)
    g_vec = acc[k].mean(0)
    print()
    print('[정지 %0.1f 초, %d 샘플]' % (still, k.sum()))
    print('  자이로 바이어스 (%.4f, %.4f, %.4f) rad/s' % tuple(bias_w))
    print('  중력 방향 |a| = %.3f m/s^2 (기대 %.3f)' % (np.linalg.norm(g_vec), G))

    # --- 자세 적분 ------------------------------------------------------
    # 시작 자세: 측정한 중력이 -z(월드) 를 향하도록 정렬
    up = g_vec / np.linalg.norm(g_vec)              # LiDAR 프레임에서 본 '위'
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(up, z); c = float(up @ z)
    if np.linalg.norm(v) < 1e-9:
        Rwb = np.eye(3)
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        Rwb = np.eye(3) + vx + vx @ vx * (1 / (1 + c))   # 월드 <- LiDAR

    P = np.zeros(3); V = np.zeros(3)
    traj = np.zeros((len(t), 3))
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if not (0 < dt < 0.05):
            traj[i] = P; continue
        # 자세
        om = (w[i] - bias_w) * dt
        th = np.linalg.norm(om)
        if th > 1e-12:
            ax = om / th
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            dR = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
            Rwb = Rwb @ dR
        # 위치
        a_w = Rwb @ acc[i] - np.array([0, 0, G])
        V += a_w * dt
        P += V * dt
        traj[i] = P

    tt = t - t[0]
    d = float(np.hypot(np.diff(traj[:, 0]), np.diff(traj[:, 1])).sum())
    print()
    print('--- IMU 단독 추측항법 결과')
    print('  수평 총거리 %10.1f m' % d)
    print('  시작-끝     %10.1f m' % math.hypot(traj[-1, 0], traj[-1, 1]))
    print('  수평 최대변위 %8.1f m' % np.hypot(traj[:, 0], traj[:, 1]).max())
    print('  z 최종      %10.1f m' % traj[-1, 2])
    print()
    for lo in range(0, int(tt[-1]), 60):
        j = np.searchsorted(tt, min(lo + 60, tt[-1])) - 1
        print('  t=%3d s   위치 (%9.1f, %9.1f) m   원점거리 %9.1f m'
              % (lo + 60, traj[j, 0], traj[j, 1], math.hypot(traj[j, 0], traj[j, 1])))

    with open(out, 'w', newline='') as f:
        wtr = csv.writer(f)
        wtr.writerow(['t', 'x', 'y', 'z'])
        step = max(1, len(t) // 5000)
        for i in range(0, len(t), step):
            wtr.writerow(['%.3f' % tt[i], '%.3f' % traj[i, 0],
                          '%.3f' % traj[i, 1], '%.3f' % traj[i, 2]])
    print()
    print('저장 ->', out)


if __name__ == '__main__':
    main()
