#!/usr/bin/env python3
"""
accel_step_check.py  (rev2) -- 시각 보간 실익 판정 + accel 실제 갱신율 확인

rev1 버그: 중앙값이 0.0 이면 파이썬에서 거짓으로 취급돼 '판정 불가'로
           빠졌다. 수정함.
rev2 추가: |Δa|=0 비율(값이 샘플마다 갱신되는지 = held 여부)과
           비영 중앙값, 추정 갱신율을 함께 출력한다.
           - Δa=0 이 많다 = accel 이 500Hz 로 신선하지 않고 유지되는 것
           - 갱신율 = 500 * (1 - 동적구간 Δa=0 비율)  [대략]

사용:
    source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
    python3 accel_step_check.py <bag.db3 또는 bag_dir>
"""
import sys
import os
import glob
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from unitree_go.msg import LowState


def find_db3(path):
    if os.path.isdir(path):
        cands = sorted(glob.glob(os.path.join(path, '*.db3')))
        if not cands:
            sys.exit('*.db3 를 %s 안에서 못 찾음' % path)
        return cands[0]
    return path


def main():
    if len(sys.argv) < 2:
        sys.exit('사용: python3 accel_step_check.py <bag.db3 또는 bag_dir>')

    db = find_db3(sys.argv[1])
    con = sqlite3.connect(db)

    row = con.execute("SELECT id FROM topics WHERE name='/lowstate'").fetchone()
    if not row:
        sys.exit('/lowstate 토픽이 이 bag 에 없음')
    tid = row[0]

    tick, acc, gyr = [], [], []
    for (data,) in con.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tid,)):
        m = deserialize_message(bytes(data), LowState)
        tick.append(int(m.tick))
        acc.append([m.imu_state.accelerometer[i] for i in range(3)])
        gyr.append([m.imu_state.gyroscope[i] for i in range(3)])
    con.close()

    if len(tick) < 10:
        sys.exit('/lowstate 표본이 너무 적음 (%d개)' % len(tick))

    tick = np.array(tick, dtype=np.int64)
    acc = np.array(acc, dtype=float)
    gyr = np.array(gyr, dtype=float)

    # 측정 순서(tick)로 정렬 + 중복 tick 제거
    order = np.argsort(tick, kind='stable')
    tick, acc, gyr = tick[order], acc[order], gyr[order]
    keep = np.concatenate(([True], np.diff(tick) > 0))
    tick, acc, gyr = tick[keep], acc[keep], gyr[keep]

    dt = np.diff(tick)                                    # ms 간격
    da = np.linalg.norm(np.diff(acc, axis=0), axis=1)     # 인접 accel 변화 |Δa|
    wn = np.linalg.norm(gyr[:-1], axis=1)                 # 그 구간 자이로 크기

    step = (dt >= 1) & (dt <= 3)          # 정상 500 Hz 스텝만 (1~3 ms)
    rest = step & (wn < 0.03)             # 정지 = 노이즈 바닥
    dyn = step & (wn > 0.30)              # 회전/보행 = 동적

    def block(name, mask):
        v = da[mask]
        if len(v) == 0:
            print('%-4s  표본 없음' % name)
            return None
        zero = float(np.mean(v == 0.0))          # 값이 그대로 유지된 비율
        nz = v[v > 0.0]
        med_nz = float(np.median(nz)) if len(nz) else 0.0
        p90 = float(np.quantile(v, .9))
        p99 = float(np.quantile(v, .99))
        print('%-4s  n=%-6d  Δa=0 %5.1f%%  비영중앙 %.4f  90%% %.4f  99%% %.4f'
              % (name, len(v), zero * 100, med_nz, p90, p99))
        return {'zero': zero, 'med_nz': med_nz, 'p90': p90}

    print('bag =', db)
    print('전체 lowstate = %d,  정상스텝(1~3ms) = %d,  dt중앙 = %d ms'
          % (len(tick), int(step.sum()), int(np.median(dt))))
    print('-' * 70)
    r = block('정지', rest)
    d = block('동적', dyn)
    print('-' * 70)

    if r is None or d is None:
        print('판정 불가: 정지 또는 동적 표본이 없음.')
        return

    # 1) accel 실제 갱신율 (동적 구간의 held 비율로 추정)
    est_hz = 500.0 * (1.0 - d['zero'])
    print('accel Δa=0 비율: 정지 %.1f%% / 동적 %.1f%%   -> 추정 갱신율 ≈ %.0f Hz'
          % (r['zero'] * 100, d['zero'] * 100, est_hz))

    # 2) 신호 대 노이즈 (90분위 기준, 중앙값은 held 때문에 0이라 못 씀)
    ratio = d['p90'] / r['p90'] if r['p90'] > 0 else float('inf')
    print('동적/정지 90분위 비 = %.1f 배' % ratio)
    print('-' * 70)

    # 3) 종합 판정
    if d['zero'] > 0.40:
        print('판정: accel 이 500Hz 로 신선하지 않고 held 됨(갱신율 %.0fHz 추정).'
              % est_hz)
        print('      -> 500Hz 가정한 2ms 보간은 전제부터 어긋남.')
        print('      -> 보간하려면 "진짜 갱신 시점"에만 맞춰야 함. 재설계 필요.')
    elif ratio < 1.5:
        print('판정: 동적 변화가 노이즈와 비슷 -> 보간 이득 미미 -> 스킵 권장')
    elif ratio < 2.5:
        print('판정: 애매 -> 다른 bag 으로 한 번 더 확인')
    else:
        print('판정: accel 이 대체로 신선하고 동적 신호가 노이즈보다 큼')
        print('      -> 시각 보간 실익 있음 -> 노드 구현 진행')


if __name__ == '__main__':
    main()
