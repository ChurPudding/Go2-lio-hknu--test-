#!/usr/bin/env python3
"""
legodom_vs_gps.py  --  다리 오도메트리 vs GPS 비교 (실외 bag 오프라인 분석)

목적
----
실외에서 다리 운동학 오도메트리가 얼마나 표류하는지 GPS 기준으로 실측한다.
"실외 개활지에서는 GPS 융합이 필수" 라는 주장의 직접 근거를 만든다.

입력 (bag 안에 이 둘만 있으면 된다)
  /lf/sportmodestate   unitree_go/msg/SportModeState   10 Hz   position[3], rpy
  /gnss                std_msgs/String (JSON)          0.5 Hz

`rosbag2_2026_07_30-16_50_28` 은 라이다·/lowstate 가 마지막 3초에만 들어와 LIO 는
못 돌리지만, 위 두 토픽은 944초 전 구간이 온전하다.

방법
----
1. 두 궤적을 각자의 시작점 기준 상대좌표로 옮긴다
2. GPS 를 ENU(동-북) 미터로 변환 (국소 평면 근사)
3. 다리 오도메트리는 로봇 자체 좌표계라 방위가 다르다. 전체 궤적에 대해
   **최적 yaw 한 개**를 최소자승으로 맞춘 뒤 비교한다 (summarize.py 와 같은 방식)
4. GPS 시각에 다리 오도메트리를 보간해 수평 오차를 낸다

주의
----
GPS 자체 오차가 HDOP 0.78 기준 약 2.4 m 다. 그보다 작은 차이는 의미 없다.
여기서 보려는 것은 수십 m 규모의 표류이므로 문제되지 않는다.

사용
----
    python3 legodom_vs_gps.py <bag_0.db3> [출력.csv]
"""
import sys
import math
import json
import struct
import sqlite3

import numpy as np

# SportModeState 필드 오프셋 (CDR encapsulation 헤더 4바이트 이후 기준)
#   stamp.sec 0, stamp.nanosec 4, error_code 8,
#   imu_state: quaternion 12, gyroscope 28, accelerometer 40, rpy 52, temperature 64
#   mode 65, progress 68, gait_type 72, foot_raise_height 76,
#   position 80, body_height 92, velocity 96, yaw_speed 108, ...
OFF_RPY = 52
OFF_POS = 80
OFF_BODY_H = 92
OFF_VEL = 96
MIN_LEN = 4 + 112          # yaw_speed 까지는 있어야 함

MLAT = 111320.0


def read_sportmode(con):
    r = con.execute("SELECT id FROM topics WHERE name='/lf/sportmodestate'").fetchone()
    if r is None:
        r = con.execute("SELECT id FROM topics WHERE name='/sportmodestate'").fetchone()
    if r is None:
        sys.exit('sportmodestate 토픽 없음')
    out = []
    short = 0
    for ts, blob in con.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (r[0],)):
        if len(blob) < MIN_LEN:
            short += 1
            continue
        le = blob[1] == 1
        E = '<' if le else '>'
        pos = struct.unpack_from(E + '3f', blob, 4 + OFF_POS)
        rpy = struct.unpack_from(E + '3f', blob, 4 + OFF_RPY)
        bh = struct.unpack_from(E + 'f', blob, 4 + OFF_BODY_H)[0]
        out.append((ts * 1e-9, pos[0], pos[1], pos[2], rpy[2], bh))
    if short:
        print('  (짧은 메시지 %d 개 건너뜀)' % short)
    return np.array(out)


def read_gnss(con):
    r = con.execute("SELECT id FROM topics WHERE name='/gnss'").fetchone()
    if r is None:
        sys.exit('/gnss 토픽 없음')
    out = []
    for ts, blob in con.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (r[0],)):
        try:
            d = json.loads(blob[8:].decode('utf-8', 'replace').strip('\x00'))
        except Exception:
            continue
        if int(d.get('fixed', 0)) != 1:
            continue
        if int(d.get('satellite_inuse', 0)) < 4:
            continue
        h = float(d.get('hdop', 0.0))
        if not (0.0 < h <= 5.0):
            continue
        out.append((ts * 1e-9, float(d['latitude']), float(d['longitude']), h))
    return np.array(out)


def path_len(P):
    return float(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])).sum())


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    db = sys.argv[1]
    outcsv = sys.argv[2] if len(sys.argv) > 2 else 'legodom_vs_gps.csv'

    con = sqlite3.connect(db)
    print('다리 오도메트리 읽는 중...')
    S = read_sportmode(con)
    print('  %d 개, %.1f 초' % (len(S), S[-1, 0] - S[0, 0]))
    print('GPS 읽는 중...')
    G = read_gnss(con)
    print('  유효 %d 개, %.1f 초' % (len(G), G[-1, 0] - G[0, 0]))

    if len(S) < 10 or len(G) < 10:
        sys.exit('데이터가 너무 적다')

    # --- 자기 검증 ------------------------------------------------------
    bh = S[:, 5]
    print()
    print('[검증] body_height 평균 %.3f m (0.25~0.40 이면 정상)' % bh.mean())
    yaw = S[:, 4]
    dyaw = np.unwrap(yaw)
    print('[검증] 누적 yaw %.1f deg' % np.degrees(dyaw[-1] - dyaw[0]))

    # --- GPS -> ENU -----------------------------------------------------
    lat0, lon0 = G[0, 1], G[0, 2]
    mlon = MLAT * math.cos(math.radians(lat0))
    gx = (G[:, 2] - lon0) * mlon
    gy = (G[:, 1] - lat0) * MLAT
    gt = G[:, 0]

    # --- 다리 오도메트리을 GPS 시각으로 보간 -----------------------------
    lx = np.interp(gt, S[:, 0], S[:, 1])
    ly = np.interp(gt, S[:, 0], S[:, 2])
    lx = lx - lx[0]
    ly = ly - ly[0]

    # --- 최적 yaw 한 개로 정렬 -------------------------------------------
    a0 = math.atan2(float((lx * gy - ly * gx).sum()),
                    float((lx * gx + ly * gy).sum()))
    rx = math.cos(a0) * lx - math.sin(a0) * ly
    ry = math.sin(a0) * lx + math.cos(a0) * ly

    err = np.hypot(rx - gx, ry - gy)

    print()
    print('=' * 62)
    print('GPS        총거리 %8.1f m   시작-끝 %6.1f m' %
          (path_len(np.stack([gx, gy], 1)), math.hypot(gx[-1], gy[-1])))
    print('다리 오도메트리 총거리 %8.1f m   시작-끝 %6.1f m' %
          (path_len(np.stack([rx, ry], 1)), math.hypot(rx[-1], ry[-1])))
    print('정렬 yaw   %+.1f deg' % math.degrees(a0))
    print('-' * 62)
    print('수평 오차   RMS %6.1f m   최대 %6.1f m   최종 %6.1f m' %
          (float(np.sqrt((err ** 2).mean())), float(err.max()), float(err[-1])))
    print('=' * 62)

    # 시간 구간별 오차 증가 추이
    print()
    print('구간별 오차 (표류가 시간에 비례하는지)')
    T = gt - gt[0]
    for lo in range(0, int(T[-1]), 120):
        m = (T >= lo) & (T < lo + 120)
        if m.sum():
            print('  %4d~%4d 초   평균 %6.1f m   최대 %6.1f m'
                  % (lo, lo + 120, err[m].mean(), err[m].max()))

    # --- CSV ------------------------------------------------------------
    import csv
    with open(outcsv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'gps_x', 'gps_y', 'leg_x', 'leg_y', 'err_m', 'hdop'])
        for i in range(len(gt)):
            w.writerow(['%.2f' % (gt[i] - gt[0]),
                        '%.2f' % gx[i], '%.2f' % gy[i],
                        '%.2f' % rx[i], '%.2f' % ry[i],
                        '%.2f' % err[i], G[i, 3]])
    print()
    print('저장 ->', outcsv)


if __name__ == '__main__':
    main()
