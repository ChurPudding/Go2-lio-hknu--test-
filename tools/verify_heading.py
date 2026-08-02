#!/usr/bin/env python3
"""verify_heading.py -- bag 을 직접 읽어 HeadingEstimator 를 검증한다.

DDS 를 거치지 않는다. bag 에서 메시지를 시간 순으로 꺼내 코어에 넣고,
결과를 오프라인 정답(`results/yaw_gps_diff.csv`)과 대조한다.

이 방식의 이점
- 재현 가능하다. 같은 입력에 항상 같은 출력
- 빠르다. 306 s bag 이 몇 초에 끝난다
- 중간 값을 마음대로 볼 수 있다

사용법
------
    python3 verify_heading.py [bag경로] [--baseline 3.0] [--tau 20.0]
"""
import json
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from unitree_go.msg import LowState, SportModeState

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from heading_core import HeadingEstimator, wrap  # noqa: E402

LOWSTATE_EVERY = 25    # 500 Hz -> 20 Hz
SPORT_EVERY = 15       # 300 Hz -> 20 Hz


def run(bag_path, **kw):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=["/lowstate", "/gnss", "/sportmodestate"]))

    est = HeadingEstimator(**kw)
    n_low = n_sp = n_gps = 0
    t0 = None
    track = []          # (t, heading_deg, offset_deg)

    while reader.has_next():
        topic, data, ts = reader.read_next()
        t = ts * 1e-9
        if t0 is None:
            t0 = t

        if topic == "/lowstate":
            n_low += 1
            if n_low % LOWSTATE_EVERY:
                continue
            m = deserialize_message(data, LowState)
            q = m.imu_state.quaternion          # [w, x, y, z]
            yaw = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]),
                             1 - 2 * (q[2] ** 2 + q[3] ** 2))
            est.push_imu(t, float(yaw))
            h = est.heading()
            if h is not None:
                track.append((t - t0, np.degrees(h),
                              np.degrees(est.offset)))

        elif topic == "/sportmodestate":
            n_sp += 1
            if n_sp % SPORT_EVERY:
                continue
            m = deserialize_message(data, SportModeState)
            est.push_velocity(t, float(m.velocity[0]), float(m.velocity[1]))

        else:  # /gnss
            try:
                d = json.loads(deserialize_message(data, String).data)
            except Exception:
                continue
            if not d.get("fixed"):
                continue
            n_gps += 1
            est.push_gps(t, float(d["latitude"]), float(d["longitude"]))

    return est, np.array(track), t0, (n_low, n_sp, n_gps)


def main(bag_path="/home/hyo/fastlio_ws/go2_outdoor_0731_1114",
         truth_csv=None, **kw):
    if truth_csv is None:
        # bag 이름별로 정답 파일을 나눈다. 섞이면 오프셋 차이가 오차로 보인다.
        tag = bag_path.rstrip('/').rsplit('/', 1)[-1]
        cand = f"results/yaw_gps_diff_{tag}.csv"
        import os
        truth_csv = cand if os.path.exists(cand) else "results/yaw_gps_diff.csv"
        print(f"정답  {truth_csv}")
    est, track, t0, counts = run(bag_path, **kw)
    n_low, n_sp, n_gps = counts

    print(f"입력  IMU {n_low}  속도 {n_sp}  GPS {n_gps}")
    print(f"설정  기선 {est.L:.1f} m, 시정수 {est.tau:.0f} s, "
          f"β 보정 {'켬' if est.use_beta else '끔'}")
    print(f"관측  채택 {est.n_obs}  기각 {est.n_reject}  "
          + "  ".join(f"{k} {v}" for k, v in est.reject_reason.items()))

    if est.offset is None:
        print("\n오프셋이 한 번도 설정되지 않았다.")
        print("게이팅 조건을 완화하거나 입력을 확인할 것.")
        return

    lg = np.array(est.log)
    lt = lg[:, 0] - t0
    lo = np.degrees(np.unwrap(lg[:, 1]))
    print(f"\n최초 관측 {lt[0]:.1f} s   수렴 "
          f"{lt[min(est.init_count, len(lt)-1)]:.1f} s")
    print(f"오프셋 {lo[0]:+.2f}° → {lo[-1]:+.2f}°"
          f"   기울기 {np.polyfit(lt, lo, 1)[0]*60:+.2f}°/분")

    # 정답과 대조
    try:
        a = np.loadtxt(truth_csv, delimiter=",", skiprows=1)
    except Exception as e:
        print(f"\n정답 CSV 를 못 읽었다: {e}")
        print("먼저 tools/yaw_gps_check.py 를 --beta 로 실행할 것.")
        return

    # 오프라인은 diff = yaw - course, 코어는 offset = course - yaw.
    # heading = yaw + offset 이 성립하려면 코어 쪽이 맞다. 부호를 맞춘다.
    tt, td = a[:, 0], -np.degrees(np.unwrap(np.radians(a[:, 3])))
    ts_, to_ = np.polyfit(tt, td, 1)
    rms_off = np.sqrt(((td - (ts_ * tt + to_)) ** 2).mean())
    print(f"\n오프라인 정답  절편 {to_:+.2f}°  기울기 {ts_*60:+.2f}°/분"
          f"  개별 잔차 RMS {rms_off:.2f}°")

    ref = ts_ * lt + to_
    err = lo - ref
    tail = err[len(err) // 3:]          # 수렴 후 구간
    print(f"\n── 대조 (추세선 대비) ──")
    print(f"전체    평균 {err.mean():+.2f}°  표준편차 {err.std():.2f}°")
    print(f"수렴후  평균 {tail.mean():+.2f}°  표준편차 {tail.std():.2f}°"
          f"  최대 {np.abs(tail).max():.2f}°")

    print("\n  시각      노드      정답추세     차이")
    step = max(1, len(lt) // 15)
    for k in range(0, len(lt), step):
        print(f"{lt[k]:7.1f} s {lo[k]:+9.2f}° {ref[k]:+9.2f}°"
              f" {err[k]:+8.2f}°")

    print()
    bias = tail.mean()          # 부호 있는 평균 = 편향. 이게 중요하다
    noise = tail.std()          # 흔들림. 시간이 지나면 상쇄된다
    print(f"편향 {bias:+.2f}°   흔들림 {noise:.2f}°")
    if abs(bias) < 2.0 and noise < 6.0:
        print(f"→ 구현이 맞다. 개별 관측 {rms_off:.2f}° → {noise:.2f}° 로")
        print("  필터가 잡음을 줄이고 있고, 계통 편향도 거의 없다.")
        print(f"  참고: 반지름 20 m 에서 {noise:.1f}° 는 횡방향 "
              f"{20*np.tan(np.radians(noise)):.2f} m."
              " GPS 측위 오차 2.4 m 보다 작다.")
    elif abs(bias) < 2.0:
        print(f"→ 편향은 없으나 흔들림 {noise:.1f}° 가 크다.")
        print("  시정수를 늘려 볼 것 (--tau 30).")
    else:
        print(f"→ 편향 {bias:+.1f}° 가 남는다. 평균을 내도 안 사라지는 종류다.")
        print("  β 보정이나 좌표계 정의를 점검할 것.")

    if len(track):
        print(f"\n방위 발행 {len(track)}회"
              f"  마지막 {track[-1][1]:+.2f}°")


if __name__ == "__main__":
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    kw = {}
    for k in ("baseline", "tau", "init_tau", "min_speed", "max_lateral"):
        if f"--{k}" in sys.argv:
            kw[k] = float(sys.argv[sys.argv.index(f"--{k}") + 1])
    if "--nobeta" in sys.argv:
        kw["beta_correct"] = False
    main(args[0] if args else
         "/home/hyo/fastlio_ws/go2_outdoor_0731_1114", **kw)
