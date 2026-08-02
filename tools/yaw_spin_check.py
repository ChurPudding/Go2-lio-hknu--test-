#!/usr/bin/env python3
"""제자리 회전 bag 으로 yaw 스케일·표류를 확인한다.

사용법:
    source /opt/ros/humble/setup.bash
    source ~/unitree_ros2/cyclonedds_ws/install/setup.bash   # unitree_go 메시지
    python3 yaw_spin_check.py ~/fastlio_ws/go2_outdoor_0731_1119
"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from unitree_go.msg import LowState


def read_lowstate(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/lowstate"]))

    t, rpy, gyro, quat = [], [], [], []
    while reader.has_next():
        _, data, ts = reader.read_next()
        m = deserialize_message(data, LowState)
        t.append(ts * 1e-9)
        rpy.append(list(m.imu_state.rpy))
        gyro.append(list(m.imu_state.gyroscope))
        quat.append(list(m.imu_state.quaternion))
    return (np.array(t), np.array(rpy), np.array(gyro), np.array(quat))


def yaw_from_quat(q):
    # unitree quaternion 순서는 [w, x, y, z]
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main(bag_path):
    t, rpy, gyro, quat = read_lowstate(bag_path)
    t = t - t[0]
    dt = np.diff(t)

    print(f"샘플 {len(t)}개, {t[-1]:.2f} s, 평균 {len(t)/t[-1]:.1f} Hz")
    print(f"dt 중앙값 {np.median(dt)*1e3:.3f} ms, 최대 {dt.max()*1e3:.1f} ms")
    print()

    # 1) rpy[2] 누적 변화량
    yaw_rpy = np.unwrap(rpy[:, 2])
    d_rpy = np.degrees(yaw_rpy[-1] - yaw_rpy[0])

    # 2) 쿼터니언에서 뽑은 yaw (rpy 와 일치하는지 교차 확인)
    yaw_q = np.unwrap(yaw_from_quat(quat))
    d_q = np.degrees(yaw_q[-1] - yaw_q[0])
    resid = np.degrees(np.abs((yaw_rpy - yaw_rpy[0]) - (yaw_q - yaw_q[0]))).max()

    # 3) 자이로 z 직접 적분
    yaw_gyro = np.concatenate([[0.0], np.cumsum(gyro[1:, 2] * dt)])
    d_gyro = np.degrees(yaw_gyro[-1])

    print(f"rpy[2] 누적      {d_rpy:9.2f}°")
    print(f"quaternion yaw   {d_q:9.2f}°   (rpy 와 최대 차 {resid:.3f}°)")
    print(f"gyro z 적분      {d_gyro:9.2f}°")
    print()

    for name, val in (("rpy[2]", d_rpy), ("gyro 적분", d_gyro)):
        for turns in (1, 2, 3):
            ref = 360.0 * turns
            if abs(abs(val) - ref) < 90:
                err = (abs(val) - ref) / ref * 100
                print(f"{name}: {turns}회전 기준 오차 {err:+.2f} %")
                break
        else:
            print(f"{name}: {abs(val):.1f}° — 회전 수가 모호하다. 육안 확인 필요")

    print()
    # 4) 정지 구간 표류 — 자이로가 거의 0 인 구간을 찾아 yaw 변화율을 본다
    speed = np.linalg.norm(gyro, axis=1)
    still = speed < 0.03
    runs, start = [], None
    for i, s in enumerate(still):
        if s and start is None:
            start = i
        elif not s and start is not None:
            if t[i - 1] - t[start] > 1.0:
                runs.append((start, i - 1))
            start = None
    if start is not None and t[-1] - t[start] > 1.0:
        runs.append((start, len(t) - 1))

    if not runs:
        print("정지 구간(1초 이상) 없음 — 표류는 주행 bag 에서 확인한다")
    else:
        print(f"정지 구간 {len(runs)}개")
        for a, b in runs:
            dur = t[b] - t[a]
            drift = np.degrees(yaw_rpy[b] - yaw_rpy[a])
            print(f"  {t[a]:6.2f}~{t[b]:6.2f} s ({dur:5.2f} s)"
                  f"  yaw {drift:+7.3f}°  →  {drift/dur*60:+7.2f}°/분")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         "/home/hyo/fastlio_ws/go2_outdoor_0731_1119")
