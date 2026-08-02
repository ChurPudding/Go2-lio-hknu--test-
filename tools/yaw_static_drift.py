#!/usr/bin/env python3
"""정지 상태의 yaw 표류를 측정한다.

원리
----
로봇이 서 있으면 순 회전량이 0 이므로 자이로 축척 오차는 원리상 기여하지
못한다. 그래도 yaw 가 움직이면 그것은 시간에 비례하는 바이어스 표류다.
축척 오차와 시간 표류를 완전히 분리하는 유일한 측정이다.

사용법
------
    python3 yaw_static_drift.py ~/fastlio_ws/go2_outdoor_0731_1114
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from unitree_go.msg import LowState, SportModeState

EVERY = 10            #     500 Hz -> 50 Hz
SPORT_EVERY = 15
STILL_SPEED = 0.05    # m/s 이 아래면 정지로 본다
MIN_WINDOW = 10.0     # s   이보다 짧은 창은 무시


def read(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=["/lowstate", "/sportmodestate"]))

    yt, yv, gz = [], [], []
    st, sv = [], []
    n_low = n_sp = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        t = ts * 1e-9
        if topic == "/lowstate":
            n_low += 1
            if n_low % EVERY:
                continue
            m = deserialize_message(data, LowState)
            q = m.imu_state.quaternion
            yt.append(t)
            yv.append(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
            gz.append(m.imu_state.gyroscope[2])
        else:
            n_sp += 1
            if n_sp % SPORT_EVERY:
                continue
            m = deserialize_message(data, SportModeState)
            st.append(t)
            sv.append(np.hypot(m.velocity[0], m.velocity[1]))
    return (np.array(yt), np.array(yv), np.array(gz),
            np.array(st), np.array(sv))


def main(path):
    yt, yv, gz, st, sv = read(path)
    t0 = min(yt[0], st[0])
    yt, st = yt - t0, st - t0
    yaw = np.unwrap(yv)

    speed = np.interp(yt, st, sv)
    still = speed < STILL_SPEED

    segs, s = [], None
    for k, v in enumerate(still):
        if v and s is None:
            s = k
        elif not v and s is not None:
            if yt[k - 1] - yt[s] >= MIN_WINDOW:
                segs.append((s, k - 1))
            s = None
    if s is not None and yt[-1] - yt[s] >= MIN_WINDOW:
        segs.append((s, len(yt) - 1))

    print(f"{len(yt)}개 샘플, {yt[-1]:.1f} s")
    print(f"정지 구간 ({MIN_WINDOW:.0f} s 이상) {len(segs)}개\n")
    if not segs:
        print("정지 구간 없음.")
        return

    print("  구간            길이   yaw 변화    기울기      자이로z 평균")
    slopes, weights = [], []
    for p, q in segs:
        t = yt[p:q + 1]
        y = np.degrees(yaw[p:q + 1] - yaw[p])
        sl, _ = np.polyfit(t, y, 1)
        resid = y - np.polyval([sl, np.polyfit(t, y, 1)[1]], t)
        se = (np.sqrt((resid ** 2).mean()) /
              (t.std() * np.sqrt(len(t)))) * 60
        print(f"{yt[p]:6.1f}~{yt[q]:6.1f} s {yt[q]-yt[p]:6.1f} s"
              f" {y[-1]:+8.3f}° {sl*60:+7.2f}±{se:.2f}°/분"
              f"  {np.degrees(gz[p:q+1].mean()):+7.3f}°/s")
        slopes.append(sl * 60)
        weights.append(yt[q] - yt[p])

    w = np.array(weights)
    avg = np.average(slopes, weights=w)
    print(f"\n가중 평균 표류  {avg:+.2f}°/분   (총 정지 {w.sum():.0f} s)")
    print(f"주행 전체에서 관측된 heading 증가  +2.17°/분")
    print()
    if abs(avg) < 0.5:
        print("→ 정지 중에는 거의 표류하지 않는다.")
        print("  주행 중 heading 오차는 회전할 때만 생긴다 = 자이로 축척 오차.")
        print(f"  보정 계수 후보: gyro_z 에 {1/1.0311:.4f} 를 곱한다.")
    elif abs(avg - 2.17) < 0.8:
        print("→ 정지 중에도 같은 비율로 표류한다 = 시간 바이어스 표류.")
        print("  축척 보정은 오히려 새 오차를 만든다. GPS 보정에만 의존할 것.")
    else:
        print("→ 중간값이다. 두 성분이 섞여 있다. 정지 시간을 늘려 재측정 권장.")

    print(f"\n참고: 총 정지 {w.sum():.0f} s 는"
          f" {'충분하다' if w.sum() > 60 else '짧다. 3분 정지 측정을 권한다'}.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         "/home/hyo/fastlio_ws/go2_outdoor_0731_1114")
