#!/usr/bin/env python3
"""
accel_diff_sim.py -- baseline vs redesign 가 LIO 에 먹이는 accel 이
                     실제로 얼마나 다른지 오프라인으로 재고 그림으로 본다.

노드/ config 를 만들지 않고, bag 만으로 두 방식을 재현한다.
  baseline : gyro 가 도착한 순간의 '최신 lowstate accel' (held 포함, 현재 노드)
             -> 도착시각(bag 수신) 기준으로 최신값을 붙인다.
  redesign : accel 이 실제로 바뀐 순간(Δa≠0)만 골라 200Hz 표본을 만들고,
             tick 으로 복원한 측정시각에서 gyro 시각으로 선형보간한다.

두 방식의 accel 차이 |a_red - a_base| 를 gyro 샘플마다 계산.
정지/동적으로 나눠 분포를 출력하고, 3장짜리 그림을 저장(+표시)한다.

시각(회전 R_LB, ACC_SCALE)은 두 방식에 똑같이 적용되므로, 차이의 크기
비교에는 영향이 없다(R_LB 는 노름 보존). 값은 원본 m/s^2 기준으로 낸다.

사용: python3 accel_diff_sim.py <bag.db3 또는 bag_dir>
"""
import sys
import os
import glob
import sqlite3

import numpy as np

import matplotlib
if not os.environ.get('DISPLAY'):
    matplotlib.use('Agg')          # 디스플레이 없으면 파일로만 저장
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from unitree_go.msg import LowState
from sensor_msgs.msg import Imu


def find_db3(path):
    if os.path.isdir(path):
        c = sorted(glob.glob(os.path.join(path, '*.db3')))
        if not c:
            sys.exit('*.db3 없음: %s' % path)
        return c[0]
    if not os.path.isfile(path):
        sys.exit('파일 없음(경로 오타?): %s' % path)
    return path


def main():
    if len(sys.argv) < 2:
        sys.exit('사용: python3 accel_diff_sim.py <bag.db3 또는 bag_dir>')
    db = find_db3(sys.argv[1])

    con = sqlite3.connect(db)
    if not con.execute("SELECT name FROM sqlite_master "
                       "WHERE type='table' AND name='topics'").fetchone():
        sys.exit('rosbag2 db 가 아님(topics 테이블 없음): %s' % db)
    tmap = dict(con.execute("SELECT name, id FROM topics").fetchall())
    for need in ('/lowstate', '/utlidar/imu'):
        if need not in tmap:
            sys.exit('%s 토픽이 bag 에 없음' % need)

    # --- /lowstate: (수신시각, tick, accel) ---
    tau_low, tick, acc = [], [], []
    for ts, (data,) in ((r[0], (r[1],)) for r in con.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tmap['/lowstate'],))):
        m = deserialize_message(bytes(data), LowState)
        tau_low.append(ts * 1e-9)
        tick.append(int(m.tick))
        acc.append([m.imu_state.accelerometer[i] for i in range(3)])

    # --- /utlidar/imu: (수신시각, header시각, gyro) ---
    tau_imu, hg, gyro = [], [], []
    for ts, (data,) in ((r[0], (r[1],)) for r in con.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tmap['/utlidar/imu'],))):
        m = deserialize_message(bytes(data), Imu)
        tau_imu.append(ts * 1e-9)
        hg.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        gyro.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    con.close()

    tau_low = np.array(tau_low); tick = np.array(tick, dtype=np.int64); acc = np.array(acc, float)
    tau_imu = np.array(tau_imu); hg = np.array(hg); gyro = np.array(gyro, float)
    if len(tick) < 10 or len(hg) < 10:
        sys.exit('표본 부족')

    # --- 클럭 오프셋 (모든 시각을 '녹화 수신시각' 축으로 옮긴다) ---
    #  L1 header:  h  = tau_imu + off_G   ->  tau = h - off_G
    #  tick clock: tick/1000 = tau_low + off_L  ->  tau = tick/1000 - off_L
    off_G = np.median(hg - tau_imu)
    off_L = np.median(tick / 1000.0 - tau_low)

    # ================= baseline: 도착 최신 held =================
    o = np.argsort(tau_low)                      # 수신시각 순
    taulow_s, acc_bytau = tau_low[o], acc[o]
    j = np.searchsorted(taulow_s, tau_imu, side='right') - 1
    j = np.clip(j, 0, len(acc_bytau) - 1)
    a_base = acc_bytau[j]                         # gyro 마다 붙는 held accel

    # ================= redesign: 200Hz 표본 보간 =================
    o2 = np.argsort(tick, kind='stable')         # tick(측정) 순
    tick_s, acc_s, taulow2 = tick[o2], acc[o2], tau_low[o2]
    keep = np.concatenate(([True], np.diff(tick_s) > 0))
    tick_s, acc_s = tick_s[keep], acc_s[keep]
    chg = np.concatenate(([True], np.any(np.diff(acc_s, axis=0) != 0, axis=1)))
    tick_fresh, a_fresh = tick_s[chg], acc_s[chg]         # 진짜 200Hz 표본
    tau_a = tick_fresh / 1000.0 - off_L                  # 측정시각(녹화축)

    tau_g = hg - off_G                            # gyro 측정시각(녹화축)
    a_red = np.empty((len(tau_g), 3))
    for k in range(3):
        a_red[:, k] = np.interp(tau_g, tau_a, a_fresh[:, k])

    # ================= 차이 =================
    diff = np.linalg.norm(a_red - a_base, axis=1)
    wn = np.linalg.norm(gyro, axis=1)
    rest = wn < 0.03
    dyn = wn > 0.30

    def stat(name, mask):
        v = diff[mask]
        if len(v) == 0:
            print('%-4s 표본 없음' % name); return
        print('%-4s n=%-6d  중앙 %.4f  90%% %.4f  99%% %.4f  최대 %.4f  (m/s^2)'
              % (name, len(v), np.median(v), np.quantile(v, .9),
                 np.quantile(v, .99), np.max(v)))

    print('bag =', os.path.basename(db))
    print('gyro %d개 / 200Hz표본 %d개 / off_G=%.3f off_L=%.3f'
          % (len(tau_g), len(tau_a), off_G, off_L))
    print('-' * 66)
    stat('정지', rest)
    stat('동적', dyn)
    if dyn.sum():
        big = np.mean(diff[dyn] > 0.5) * 100
        print('-' * 66)
        print('동적에서 baseline↔redesign 차이가 0.5 m/s^2 넘는 비율: %.1f%%' % big)
        print('해석: 이 값이 작으면 LIO 입력이 사실상 안 바뀜 -> 재설계 실익 적음')
        print('      크면 -> 노드 만들고 Point-LIO A/B 돌릴 값어치 있음')

    # ================= 그림 =================
    fig = plt.figure(figsize=(11, 9))

    # (1) 차이 히스토그램 (정지 vs 동적)
    ax1 = fig.add_subplot(2, 2, 1)
    hi = np.quantile(diff[dyn], .99) if dyn.sum() else 1.0
    edges = np.linspace(0, max(hi, 0.1), 60)
    if rest.sum():
        ax1.hist(diff[rest], edges, alpha=.6, label='rest', color='tab:blue')
    if dyn.sum():
        ax1.hist(diff[dyn], edges, alpha=.6, label='dynamic', color='tab:red')
    ax1.set_yscale('log')
    ax1.set_xlabel('|a_redesign - a_baseline|  [m/s^2]')
    ax1.set_ylabel('count (log)')
    ax1.set_title('LIO input difference: baseline vs redesign')
    ax1.legend(); ax1.grid(True, alpha=.3)

    # (2) 누적분포(CDF) — 동적
    ax2 = fig.add_subplot(2, 2, 2)
    if dyn.sum():
        v = np.sort(diff[dyn])
        cdf = np.arange(1, len(v) + 1) / len(v)
        ax2.plot(v, cdf, color='tab:red')
        ax2.axvline(0.5, ls='--', color='gray')
        ax2.set_xlabel('diff [m/s^2] (dynamic)')
        ax2.set_ylabel('CDF')
        ax2.set_xlim(0, np.quantile(v, .995))
        ax2.set_title('how often the input actually changes')
        ax2.grid(True, alpha=.3)

    # (3) 시간 확대 — 계단(held) vs 보간, X축 accel
    ax3 = fig.add_subplot(2, 1, 2)
    ip = int(np.argmax(wn)) if len(wn) else 0
    t0, t1 = tau_g[ip] - 0.25, tau_g[ip] + 0.25
    gm = (tau_g >= t0) & (tau_g <= t1)
    fm = (tau_a >= t0) & (tau_a <= t1)
    base = tau_g[ip]
    if gm.sum() > 2:
        ax3.step(tau_g[gm] - base, a_base[gm, 0], where='post',
                 color='tab:blue', label='baseline (held, X)', lw=1.5)
        ax3.plot(tau_g[gm] - base, a_red[gm, 0], '-',
                 color='tab:red', label='redesign (interp, X)', lw=1.5)
    if fm.sum():
        ax3.plot(tau_a[fm] - base, a_fresh[fm, 0], 'k.', ms=8,
                 label='true 200Hz samples (X)')
    ax3.set_xlabel('time [s] (0 = peak-motion instant)')
    ax3.set_ylabel('accel X [m/s^2]')
    ax3.set_title('zoom at highest-motion window: staircase vs interpolation')
    ax3.legend(loc='best'); ax3.grid(True, alpha=.3)

    fig.tight_layout()
    out = 'accel_diff_%s.png' % os.path.splitext(os.path.basename(db))[0]
    fig.savefig(out, dpi=130)
    print('\n그림 저장: %s' % os.path.abspath(out))
    if os.environ.get('DISPLAY'):
        plt.show()


if __name__ == '__main__':
    main()
