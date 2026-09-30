#!/usr/bin/env python3
"""
gyro_bias_check.py
------------------
bag 의 정지 구간에서 자이로 바이어스를 측정한다.

목적:
  Point-LIO 가 정지 중에도 yaw 를 약 -13 deg/s (= -0.227 rad/s) 로
  계속 회전시키는 현상의 원인이
      (A) 센서(자이로 바이어스)  -> 세 LIO 모두에 영향, fix 노드 수정으로 해결
      (B) Point-LIO 내부(중력정렬/extrinsic) -> Point-LIO 만의 문제
  중 어느 쪽인지 가른다.

  판정:  |L1 자이로 z 평균| 이 0.227 rad/s 근처 -> (A)
         L1·몸통 둘 다 0 근처                   -> (B)

읽는 토픽 (모두 bag 안에 있음, 로봇 연결 불필요):
  /utlidar/imu     sensor_msgs/Imu        L1 IMU (Point-LIO 입력의 자이로 원본)
  /lowstate        unitree_go/msg/LowState 몸통 IMU (fix 노드의 가속도계 출처)
  /sportmodestate  로봇 속도·높이 (정지 구간 자동 탐색용)

사용법:
  unset CYCLONEDDS_URI
  export ROS_DOMAIN_ID=99
  source /opt/ros/humble/setup.bash
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash    # LowState 타입에 필요

  # 자동으로 가장 긴 정지 구간을 찾아서 측정
  python3 gyro_bias_check.py ~/data/bags/lio_test_bag_loop_run2 --auto

  # 구간을 직접 지정 (bag 시작 기준 초)
  python3 gyro_bias_check.py ~/data/bags/lio_test_bag_loop_run2 --t0 30 --t1 50

옵션:
  --auto        /sportmodestate 로 기상 후 가장 긴 정지 구간을 자동 선택
  --t0 --t1     구간 직접 지정 [s]
  --expect      비교 기준 각속도 [rad/s] (기본 0.227 = 13 deg/s)
"""

import argparse
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

T_IMU = "/utlidar/imu"
T_LOW = "/lowstate"
T_SMS = "/sportmodestate"

R2D = 180.0 / math.pi


# ────────────────────────────────────────────────────────────
# bag 읽기
# ────────────────────────────────────────────────────────────
def resolve_bag(path):
    if os.path.isdir(path):
        for f in sorted(os.listdir(path)):
            if f.endswith(".db3"):
                return os.path.join(path, f)
        sys.exit(f"[에러] {path} 안에 .db3 가 없습니다")
    return path


def read_bag(db3, wanted):
    """wanted 에 든 토픽만 읽어 {토픽: [(t_rel, msg), ...]} 로 돌려준다."""
    storage = rosbag2_py.StorageOptions(uri=db3, storage_id="sqlite3")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, rosbag2_py.ConverterOptions("", ""))

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    have, missing = [], []
    for w in wanted:
        (have if w in types else missing).append(w)
    if missing:
        print(f"[경고] bag 에 없는 토픽: {missing}")
    if T_IMU not in have:
        sys.exit(f"[에러] {T_IMU} 가 없으면 진단할 수 없습니다")

    cls = {w: get_message(types[w]) for w in have}
    out = {w: [] for w in have}

    t0 = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in out:
            continue
        t = t_ns * 1e-9
        if t0 is None:
            t0 = t
        out[topic].append((t - t0, deserialize_message(data, cls[topic])))
    return out


# ────────────────────────────────────────────────────────────
# 정지 구간 자동 탐색
# ────────────────────────────────────────────────────────────
def find_still(sms, v_th=0.03, min_len=5.0):
    """
    /sportmodestate 로 '서 있고 + 안 움직이는' 가장 긴 구간을 찾는다.
    반환: (t0, t1)
    """
    if not sms:
        return None

    t = np.array([r[0] for r in sms])
    h = np.array([float(m.body_height) for _, m in sms])
    v = np.array([math.hypot(float(m.velocity[0]), float(m.velocity[1]))
                  for _, m in sms])

    h_th = h.min() + 0.8 * (h.max() - h.min())      # 기상 판정
    ok = (h >= h_th) & (v < v_th)                    # 서 있고 정지

    best = (0.0, None, None)
    i = 0
    n = len(ok)
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and ok[j + 1]:
            j += 1
        length = t[j] - t[i]
        if length > best[0]:
            best = (length, t[i], t[j])
        i = j + 1

    if best[1] is None or best[0] < min_len:
        return None
    # 구간 양 끝 0.5s 는 버린다 (전이 구간 오염 방지)
    return (best[1] + 0.5, best[2] - 0.5)


# ────────────────────────────────────────────────────────────
# 통계
# ────────────────────────────────────────────────────────────
def stats(name, w, t, expect):
    """w: (N,3) 각속도 [rad/s], t: (N,) 시각 [s]"""
    if len(w) < 10:
        print(f"  {name}: 샘플 부족 ({len(w)}개) — 건너뜀\n")
        return None

    dur = t[-1] - t[0]
    mean = w.mean(axis=0)
    std = w.std(axis=0)

    # 전반/후반 평균 — 바이어스가 상수인지 확인
    half = len(w) // 2
    m1, m2 = w[:half].mean(axis=0), w[half:].mean(axis=0)

    # 적분 = 정지 구간 동안 자이로만으로 예측되는 각도 변화
    ang = np.trapezoid(w, t, axis=0) * R2D if hasattr(np, "trapezoid") \
        else np.trapz(w, t, axis=0) * R2D

    print(f"  {name}   샘플 {len(w)}개, {dur:.1f}s, {len(w)/dur:.0f} Hz")
    print(f"    평균 [rad/s]  x {mean[0]:+.5f}   y {mean[1]:+.5f}   z {mean[2]:+.5f}")
    print(f"    평균 [deg/s]  x {mean[0]*R2D:+.3f}   y {mean[1]*R2D:+.3f}   z {mean[2]*R2D:+.3f}")
    print(f"    표준편차[deg/s] x {std[0]*R2D:.3f}   y {std[1]*R2D:.3f}   z {std[2]*R2D:.3f}")
    print(f"    전반 z {m1[2]*R2D:+.3f} / 후반 z {m2[2]*R2D:+.3f} deg/s"
          f"   (차이 {abs(m1[2]-m2[2])*R2D:.3f} — 작으면 상수 바이어스)")
    print(f"    적분 각도변화 x {ang[0]:+.1f}  y {ang[1]:+.1f}  z {ang[2]:+.1f} deg")
    print(f"    |z 평균| = {abs(mean[2]):.5f} rad/s   (기준 {expect:.3f} rad/s "
          f"대비 {100*abs(mean[2])/expect:.1f} %)")
    print()
    return mean


def accel_stats(name, a):
    if len(a) < 10:
        return
    mean = a.mean(axis=0)
    norm = np.linalg.norm(mean)
    # 중력이 어느 축을 향하는지
    axis = int(np.argmax(np.abs(mean)))
    sign = "+" if mean[axis] > 0 else "-"
    print(f"  {name} 가속도 평균 [m/s^2]  x {mean[0]:+.3f}  y {mean[1]:+.3f}  z {mean[2]:+.3f}")
    print(f"    크기 {norm:.3f}  (중력 9.807 대비 {100*norm/9.807:.1f} %)"
          f"   최대축 = {sign}{'xyz'[axis]}")
    print()


# ────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--expect", type=float, default=0.227,
                    help="비교 기준 각속도 [rad/s] (기본 0.227 = 13 deg/s)")
    args = ap.parse_args()

    data = read_bag(resolve_bag(args.bag), [T_IMU, T_LOW, T_SMS])
    sms = data.get(T_SMS, [])

    # ── 구간 결정 ────────────────────────────────────────
    if args.t0 is not None and args.t1 is not None:
        t0, t1 = args.t0, args.t1
        how = "직접 지정"
    else:
        found = find_still(sms)
        if found is None:
            sys.exit("[에러] 정지 구간 자동 탐색 실패. --t0 --t1 로 직접 지정하세요.")
        t0, t1 = found
        how = "자동 탐색 (/sportmodestate)"

    print("=" * 62)
    print(f"bag       : {args.bag}")
    print(f"정지 구간 : {t0:.1f} ~ {t1:.1f} s  ({t1-t0:.1f}s)   [{how}]")
    print("=" * 62)

    # ── 구간 정지 검증 ───────────────────────────────────
    if sms:
        sel = [(t, m) for t, m in sms if t0 <= t <= t1]
        if sel:
            v = np.array([math.hypot(float(m.velocity[0]), float(m.velocity[1]))
                          for _, m in sel])
            h = np.array([float(m.body_height) for _, m in sel])
            print(f"[검증] 구간 내 속도 |v|  평균 {v.mean():.4f}  최대 {v.max():.4f} m/s")
            print(f"       body_height      평균 {h.mean():.3f}  변동 {h.max()-h.min():.3f} m")
            if v.max() > 0.10:
                print("       ** 경고: 최대 속도가 0.10 m/s 를 넘습니다. 정지 구간이 아닐 수 있습니다. **")
            print()

    # ── L1 IMU ───────────────────────────────────────────
    print("[1] L1 IMU  (/utlidar/imu) — Point-LIO 가 실제로 쓰는 자이로")
    sel = [(t, m) for t, m in data[T_IMU] if t0 <= t <= t1]
    w_l1 = np.array([[m.angular_velocity.x, m.angular_velocity.y,
                      m.angular_velocity.z] for _, m in sel])
    t_l1 = np.array([t for t, _ in sel])
    m_l1 = stats("L1 자이로", w_l1, t_l1, args.expect)
    a_l1 = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                      m.linear_acceleration.z] for _, m in sel])
    accel_stats("L1", a_l1)

    # ── 몸통 IMU ─────────────────────────────────────────
    m_bd = None
    if T_LOW in data:
        print("[2] 몸통 IMU  (/lowstate) — 정상 기준 (실험 2에서 상관계수 0.83)")
        sel = [(t, m) for t, m in data[T_LOW] if t0 <= t <= t1]
        w_bd = np.array([[float(m.imu_state.gyroscope[0]),
                          float(m.imu_state.gyroscope[1]),
                          float(m.imu_state.gyroscope[2])] for _, m in sel])
        t_bd = np.array([t for t, _ in sel])
        m_bd = stats("몸통 자이로", w_bd, t_bd, args.expect)
        a_bd = np.array([[float(m.imu_state.accelerometer[0]),
                          float(m.imu_state.accelerometer[1]),
                          float(m.imu_state.accelerometer[2])] for _, m in sel])
        accel_stats("몸통", a_bd)
    else:
        print("[2] /lowstate 없음 — 몸통 IMU 비교 생략\n")

    # ── 판정 ─────────────────────────────────────────────
    print("=" * 62)
    print("판정")
    print("=" * 62)
    if m_l1 is None:
        return
    z = abs(m_l1[2])
    ratio = z / args.expect

    print(f"  Point-LIO 정지 드리프트 : {args.expect:.3f} rad/s "
          f"({args.expect*R2D:.1f} deg/s)")
    print(f"  L1 자이로 z 바이어스     : {z:.5f} rad/s ({z*R2D:.2f} deg/s)")
    if m_bd is not None:
        print(f"  몸통 자이로 z 바이어스   : {abs(m_bd[2]):.5f} rad/s "
              f"({abs(m_bd[2])*R2D:.2f} deg/s)")
    print()

    if ratio > 0.5:
        print("  -> (A) 센서 원인. L1 자이로 z 바이어스가 드리프트를 대부분 설명합니다.")
        print("     대책: l1_imu_fix.py 에서 정지 중 자이로 평균을 빼는 바이어스 보정 추가.")
        print("     영향: 세 LIO 전부 해당 -> 매트릭스 측정 전에 고쳐야 합니다.")
    elif ratio > 0.1:
        print("  -> (A/B 혼합). 자이로 바이어스가 일부만 설명합니다.")
        print("     바이어스 보정으로 줄어들긴 하나 나머지 원인이 따로 있습니다.")
    else:
        print("  -> (B) 알고리즘 원인. 자이로는 정상인데 Point-LIO 가 회전합니다.")
        print("     의심 지점: 중력 정렬(gravity align), extrinsic_R, 점군 정합 실패.")
        print("     영향: Point-LIO 고유 문제일 가능성 -> 매트릭스는 그대로 진행 가능.")
    print()
    print("  참고: 부호는 프레임 정의(Point-LIO 출력 roll 이 약 180도)로 뒤집힐 수")
    print("        있으므로 크기로 판정합니다.")


if __name__ == "__main__":
    main()
