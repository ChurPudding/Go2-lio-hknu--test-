#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
yaw_check.py — 제자리 회전 구간에서 요(yaw) 추정이 얼마나 어긋나는지 본다

  세 가지 요 값을 같은 시간축에 올려 비교한다.
    A. /utlidar/robot_odom      다리 오도메트리가 계산한 자세
    B. /utlidar/imu  자이로 z    적분값 (정지 구간에서 바이어스 제거)
    C. /sportmodestate rpy[2]    로봇 내부 융합 자세

  A 와 B·C 가 갈리면 다리 오도메트리의 회전 추정이 원인이다.
  B 와 C 가 서로 맞으면 두 독립 경로가 같은 답을 낸 것이므로 신뢰도가 높다.

  주의: L1 IMU 는 물리적으로 기울어져 있다. 자이로 z 가 진짜 연직축이 아닐 수
        있으므로, 회전 구간마다 적분값과 |ω| 적분을 함께 출력해 축을 검증한다.
        둘이 비슷하면 z 가 회전축이 맞다.

  사용: python3 yaw_check.py [bag] [--png 출력경로]
"""

import argparse
import math
import os

import numpy as np
import yaml

ODOM = '/utlidar/robot_odom'
IMU = '/utlidar/imu'
SPORT = '/sportmodestate'

TURN_RATE = math.radians(20.0)   # 이 각속도를 넘으면 회전 중으로 본다
STOP_SPEED = 0.05                # m/s


def detect_storage(bag_dir):
    meta = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.exists(meta):
        with open(meta) as f:
            m = yaml.safe_load(f)
        try:
            return m['rosbag2_bagfile_information']['storage_identifier']
        except Exception:
            pass
    for f in os.listdir(bag_dir):
        if f.endswith('.mcap'):
            return 'mcap'
    return 'sqlite3'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def collect(bag, storage, topics):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=storage), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}

    cls = {}
    for t in topics:
        if t not in types:
            print(f'! {t} 가 bag 에 없습니다 — 건너뜁니다')
            continue
        try:
            cls[t] = get_message(types[t])
        except Exception:
            print(f'! {t} 의 타입 {types[t]} 를 불러올 수 없습니다 — 건너뜁니다')
            print(f'  (해당 메시지 패키지를 source 하면 이 항목도 비교에 들어갑니다)')

    avail = list(cls)
    if not avail:
        return {}, types
    r.set_filter(StorageFilter(topics=avail))

    out = {t: [] for t in avail}
    while r.has_next():
        name, data, t_ns = r.read_next()
        out[name].append((t_ns * 1e-9, deserialize_message(data, cls[name])))
    return out, types


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', nargs='?',
                    default=os.path.expanduser('~/data/bags/indoor/floor_0805_1720'))
    ap.add_argument('--png',
                    default=os.path.expanduser('~/fastlio_ws/results/yaw_check.png'))
    args = ap.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import font_manager
    for nm in ('NanumGothic', 'NanumBarunGothic', 'Noto Sans CJK KR'):
        if any(f.name == nm for f in font_manager.fontManager.ttflist):
            matplotlib.rcParams['font.family'] = nm
            matplotlib.rcParams['axes.unicode_minus'] = False
            break
    import matplotlib.pyplot as plt

    bag = args.bag
    storage = detect_storage(bag)
    print(f'bag: {bag}\n')
    data, types = collect(bag, storage, [ODOM, IMU, SPORT])
    if ODOM not in data:
        print(f'{ODOM} 을 읽지 못했습니다. 비교할 수 없습니다.')
        return

    # --- A. 다리 오도메트리 요 ---------------------------------------
    ot = np.array([t for t, _ in data[ODOM]])
    oyaw = np.unwrap(np.array([yaw_of(m.pose.pose.orientation) for _, m in data[ODOM]]))
    oxy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y]
                    for _, m in data[ODOM]])
    k = np.argsort(ot)
    ot, oyaw, oxy = ot[k], oyaw[k], oxy[k]
    t0 = ot[0]
    ot -= t0
    oyaw -= oyaw[0]

    # 속도와 각속도
    dt = np.gradient(ot)
    dt[dt <= 0] = 1e-6
    speed = np.r_[0, np.hypot(*np.diff(oxy, axis=0).T)] / dt
    orate = np.gradient(oyaw, ot)

    # --- B. 자이로 z 적분 ---------------------------------------------
    if IMU not in data:
        print(f'{IMU} 가 없습니다.')
        return
    it = np.array([t for t, _ in data[IMU]]) - t0
    gz = np.array([m.angular_velocity.z for _, m in data[IMU]])
    gx = np.array([m.angular_velocity.x for _, m in data[IMU]])
    gy = np.array([m.angular_velocity.y for _, m in data[IMU]])
    k = np.argsort(it)
    it, gz, gx, gy = it[k], gz[k], gx[k], gy[k]

    # 자이로는 LiDAR 프레임 측정값이다. R_BL 로 몸통 프레임으로 돌린다.
    #   R_LB[2,2] = -0.9647 → 라이다 z 가 연직에서 15.3도 기울고 부호도 반대.
    #   생짜 z 를 쓰면 회전량을 3.5% 적게 읽는다.
    try:
        from go2_calib import R_BL
        w_lidar = np.stack([gx, gy, gz], axis=1)
        w_body = w_lidar @ R_BL.T
        gx, gy, gz = w_body[:, 0], w_body[:, 1], w_body[:, 2]
        print('자이로 좌표변환   : R_BL 적용 (LiDAR → base_link)')
    except Exception as e:
        print(f'! go2_calib 를 불러오지 못했습니다 — 생짜 z 를 씁니다 ({e})')
        print('  이 경우 회전량이 약 3.5% 적게 나옵니다.')

    # 바이어스: 로봇이 서 있고 회전도 없는 구간의 중앙값
    still = (np.interp(it, ot, speed) < STOP_SPEED) & \
            (np.abs(np.interp(it, ot, orate)) < math.radians(2))
    if still.sum() < 50:
        print('! 정지 구간이 거의 없습니다. 바이어스 추정이 불안정합니다.')
        bias = float(np.median(gz))
    else:
        bias = float(np.median(gz[still]))
    print(f'자이로 z 바이어스 : {math.degrees(bias):+.4f} deg/s'
          f'   (정지 샘플 {int(still.sum()):,}개)')

    idt = np.gradient(it)
    idt[idt <= 0] = 1e-6

    # 자이로 z 축 방향 판별: 오도메트리 각속도와 상관을 보고 부호를 정한다.
    # L1 IMU 는 물리적으로 기울어져 실려 있어 z 가 아래를 향할 수 있다.
    gz_c = gz - bias
    corr = float(np.dot(np.interp(ot, it, gz_c), orate))
    sign = 1.0 if corr >= 0 else -1.0
    print(f'자이로 z 축 방향  : {"정방향" if sign > 0 else "역방향 (부호 뒤집음)"}'
          f'   (상관 {corr:+.1f})')

    gz_c = gz_c * sign
    gyaw = np.cumsum(gz_c * idt)
    gmag = np.cumsum(np.sqrt(gx**2 + gy**2 + gz_c**2) * idt)

    # 정지 구간에서 바이어스 제거 후 남은 표류량 — 적분 신뢰도의 척도
    if still.sum() > 50:
        resid = float(np.median(np.abs(gz_c[still])))
        print(f'정지 중 잔여 각속도: {math.degrees(resid):.4f} deg/s'
              f'   (전체 {ot[-1]:.0f}초 적분 시 최대 {math.degrees(resid)*ot[-1]:.1f} deg 표류)')

    # --- C. 내부 융합 요 ------------------------------------------------
    st, syaw = None, None
    if SPORT in data and data[SPORT]:
        try:
            st = np.array([t for t, _ in data[SPORT]]) - t0
            syaw = np.unwrap(np.array([m.imu_state.rpy[2] for _, m in data[SPORT]]))
            k = np.argsort(st)
            st, syaw = st[k], syaw[k]
            syaw -= syaw[0]
        except Exception as e:
            print(f'! {SPORT} 에서 rpy 를 못 읽었습니다: {e}')
            st, syaw = None, None

    # --- 전체 비교 -------------------------------------------------------
    g_at_o = np.interp(ot, it, gyaw)
    print(f'\n총 회전량')
    print(f'  A 다리 오도메트리 : {math.degrees(oyaw[-1] - oyaw[0]):+9.2f} deg')
    print(f'  B 자이로 z 적분   : {math.degrees(g_at_o[-1] - g_at_o[0]):+9.2f} deg')
    if syaw is not None:
        print(f'  C 내부 융합       : {math.degrees(syaw[-1] - syaw[0]):+9.2f} deg')

    # --- 회전 구간별 -----------------------------------------------------
    turning = np.abs(orate) > TURN_RATE
    segs, i = [], 0
    n = len(ot)
    while i < n:
        if turning[i]:
            j = i
            while j + 1 < n and turning[j + 1]:
                j += 1
            if ot[j] - ot[i] > 0.7:
                segs.append((i, j))
            i = j + 1
        else:
            i += 1

    print(f'\n회전 구간 {len(segs)}개  (|각속도| > 20 deg/s, 0.7초 이상)\n')
    print('   구간[s]        A 다리      B 자이로     차이      A/B     |w|적분')
    print('  ' + '-' * 68)
    rows = []
    for i, j in segs:
        a = math.degrees(oyaw[j] - oyaw[i])
        b = math.degrees(np.interp(ot[j], it, gyaw) - np.interp(ot[i], it, gyaw))
        w = math.degrees(np.interp(ot[j], it, gmag) - np.interp(ot[i], it, gmag))
        ratio = a / b if abs(b) > 1e-6 else float('nan')
        rows.append((ot[i], ot[j], a, b, a - b, ratio, w))
        print(f'  {ot[i]:6.1f}~{ot[j]:6.1f}  {a:+9.2f}  {b:+9.2f}  {a-b:+8.2f}'
              f'  {ratio:7.4f}  {w:8.2f}')

    if rows:
        ratios = [r[5] for r in rows if np.isfinite(r[5])]
        diffs = [r[4] for r in rows]
        print('  ' + '-' * 68)
        print(f'  누적 차이 {sum(diffs):+.2f} deg'
              f'   ·  A/B 평균 {np.mean(ratios):.4f}'
              f'   ·  편차 {np.std(ratios):.4f}')

    # --- 그림 -----------------------------------------------------------
    fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=True,
                           gridspec_kw={'height_ratios': [2, 1]})
    ax[0].plot(ot, np.degrees(oyaw), label='A 다리 오도메트리', lw=1.4)
    ax[0].plot(it, np.degrees(gyaw), label='B 자이로 z 적분', lw=1.2)
    if syaw is not None:
        ax[0].plot(st, np.degrees(syaw), label='C 내부 융합', lw=1.0, alpha=0.8)
    for i, j in segs:
        ax[0].axvspan(ot[i], ot[j], color='orange', alpha=0.15)
        ax[1].axvspan(ot[i], ot[j], color='orange', alpha=0.15)
    ax[0].set_ylabel('yaw [deg]')
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    ax[0].set_title('요 추정 비교 — 주황색이 제자리 회전 구간')

    ax[1].plot(ot, np.degrees(oyaw - g_at_o), color='crimson', lw=1.2)
    ax[1].axhline(0, color='gray', lw=0.8)
    ax[1].set_ylabel('A - B [deg]')
    ax[1].set_xlabel('시간 [s]')
    ax[1].grid(alpha=0.3)

    os.makedirs(os.path.dirname(args.png), exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.png, dpi=110)
    print(f'\n그림 저장: {args.png}')


if __name__ == '__main__':
    main()
