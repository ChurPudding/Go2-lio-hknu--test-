#!/usr/bin/env python3
"""
accel_rate_confirm.py -- 본체 accel 의 실제 갱신율을 '직접' 측정해 확인한다.

지금까지의 202Hz 는 held 비율(59.6%)에서 역산한 '추정'이었다.
여기서는 두 가지를 직접 잰다.
  (1) 직접 카운트: accel 이 새 값으로 바뀐 횟수 / 시간 = 진짜 갱신율
  (2) run length : 같은 값이 몇 개 메시지에 걸쳐 유지되는지
                   500Hz ÷ 200Hz = 2.5 이므로, 200Hz 라면 2와 3에 몰려야 함
두 관점이 같은 답을 주면 확정. 어긋나면 200Hz 가정이 틀린 것.

gyro 도 같이 재서, 본체 IMU 전체가 200Hz 인지 accel 만 그런지 본다.

사용: python3 accel_rate_confirm.py <bag.db3 또는 bag_dir>
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
        c = sorted(glob.glob(os.path.join(path, '*.db3')))
        if not c:
            sys.exit('*.db3 없음: %s' % path)
        return c[0]
    if not os.path.isfile(path):
        sys.exit('파일 없음(경로 오타?): %s' % path)     # 지난번 오타 가드
    return path


def hist(v, vals):
    return ' '.join('%d개:%d' % (x, int(np.sum(v == x))) for x in vals)


def main():
    if len(sys.argv) < 2:
        sys.exit('사용: python3 accel_rate_confirm.py <bag.db3 또는 bag_dir>')
    db = find_db3(sys.argv[1])

    con = sqlite3.connect(db)
    if not con.execute("SELECT name FROM sqlite_master "
                       "WHERE type='table' AND name='topics'").fetchone():
        sys.exit('rosbag2 db 가 아님(topics 테이블 없음): %s' % db)
    row = con.execute("SELECT id FROM topics WHERE name='/lowstate'").fetchone()
    if not row:
        sys.exit('/lowstate 토픽 없음')
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

    tick = np.array(tick, dtype=np.int64)
    acc = np.array(acc, float)
    gyr = np.array(gyr, float)

    # 측정 순서(tick)로 정렬 + 중복 tick 제거
    o = np.argsort(tick, kind='stable')
    tick, acc, gyr = tick[o], acc[o], gyr[o]
    keep = np.concatenate(([True], np.diff(tick) > 0))
    tick, acc, gyr = tick[keep], acc[keep], gyr[keep]

    dur = (tick[-1] - tick[0]) / 1000.0        # 초 (tick = ms)

    def analyze(name, sig):
        chg = np.any(np.diff(sig, axis=0) != 0, axis=1)   # 값이 바뀐 스텝
        n_chg = int(chg.sum())
        idx = np.flatnonzero(chg)
        if len(idx) < 3:
            print('\n=== %s ===  변화 표본 부족 (거의 안 바뀜)' % name)
            return
        runs = np.diff(idx)                    # 메시지 몇 개마다 새 값
        r = runs[runs <= 12]
        rate_count = n_chg / dur               # (1) 직접 카운트
        rate_run = 500.0 / np.mean(r)          # (2) run length 로부터
        print('\n=== %s ===' % name)
        print(' (1) 직접 카운트 : %d회 변화 / %.1f s = %.1f Hz'
              % (n_chg, dur, rate_count))
        print(' (2) run length  : 평균 %.2f개  -> %.1f Hz' % (np.mean(r), rate_run))
        print('     run 분포     : %s' % hist(r, [1, 2, 3, 4, 5, 6]))

    print('bag =', db)
    print('lowstate = %d개, 길이 %.1f s' % (len(tick), dur))
    analyze('accel', acc)
    analyze('gyro ', gyr)
    print('\n판정 기준:')
    print(' - (1)과 (2)가 둘 다 200 근처 + bag 마다 재현 -> 200Hz 확정')
    print(' - run 분포가 2,3 에 몰리면(2.5 평균) -> 500/200 다운샘플 신호, 강한 증거')
    print(' - 흩어지거나 다른 값이면 -> 200Hz 추정이 틀린 것')


if __name__ == '__main__':
    main()
