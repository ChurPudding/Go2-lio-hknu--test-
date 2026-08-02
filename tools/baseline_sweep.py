#!/usr/bin/env python3
"""기선 길이에 따른 방위각 잡음을 잰다.

목적
----
실시간 노드는 미래 GPS 점을 쓸 수 없으므로 기선이 길수록 heading 이 늦게
나온다. 반대로 짧으면 잡음이 커진다. 그 맞교환을 실측으로 정한다.

이론상 각도 오차는 기선 L 에 반비례한다 (GPS 위치 오차 / L). 다만 수신기가
반송파 평활화를 하므로 실제로는 다를 수 있다. 재봐야 안다.

사용법
------
    python3 baseline_sweep.py ~/fastlio_ws/go2_outdoor_0731_1114
"""
import json
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from unitree_go.msg import LowState, SportModeState

BASELINES = [1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 7.0, 10.0]
MAX_SPAN = 40.0
MAX_BOW_RATIO = 0.20   #     현 길이 대비 이 비율까지 직선으로 본다
MIN_SPEED = 0.20
MAX_LATERAL = 0.40     # m/s β 보정을 켜므로 완화
LOWSTATE_EVERY = 25
SPORT_EVERY = 15
R_EARTH = 6378137.0


def read(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=["/lowstate", "/gnss", "/sportmodestate"]))

    yt, yv, gt, lat, lon, st, vx, vy = [], [], [], [], [], [], [], []
    n_low = n_sp = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        t = ts * 1e-9
        if topic == "/lowstate":
            n_low += 1
            if n_low % LOWSTATE_EVERY:
                continue
            m = deserialize_message(data, LowState)
            q = m.imu_state.quaternion
            yt.append(t)
            yv.append(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
        elif topic == "/sportmodestate":
            n_sp += 1
            if n_sp % SPORT_EVERY:
                continue
            m = deserialize_message(data, SportModeState)
            st.append(t)
            vx.append(m.velocity[0])
            vy.append(m.velocity[1])
        else:
            try:
                d = json.loads(deserialize_message(data, String).data)
                if not d.get("fixed"):
                    continue
                gt.append(t)
                lat.append(d["latitude"])
                lon.append(d["longitude"])
            except Exception:
                pass
    return tuple(np.array(x) for x in
                 (yt, yv, gt, lat, lon, st, vx, vy))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def evaluate(L, gt, E, N, yt, yaw_u, st, vx, vy, beta_correct):
    """기선 L 로 방위 관측을 만들고 잔차를 돌려준다."""
    rows = []
    for i in range(len(gt)):
        j = i + 1
        while j < len(gt):
            d = np.hypot(E[j] - E[i], N[j] - N[i])
            if gt[j] - gt[i] > MAX_SPAN:
                j = -1
                break
            if d >= L:
                break
            j += 1
        if j < 0 or j >= len(gt):
            continue

        dE, dN = E[j] - E[i], N[j] - N[i]
        chord = np.hypot(dE, dN)
        if j - i > 1:
            mid = slice(i + 1, j)
            bow = np.abs((E[mid] - E[i]) * dN - (N[mid] - N[i]) * dE) / chord
            if bow.max() > MAX_BOW_RATIO * chord:
                continue

        sel = (st >= gt[i]) & (st <= gt[j])
        if sel.sum() < 3:
            continue
        mvx = vx[sel].mean()
        if mvx < MIN_SPEED:
            continue
        if np.abs(vy[sel]).mean() > MAX_LATERAL:
            continue

        course = np.arctan2(dN, dE)
        if beta_correct:
            course -= np.arctan2(vy[sel].mean(), mvx)

        tm = 0.5 * (gt[i] + gt[j])
        yaw = np.interp(tm, yt, yaw_u)
        rows.append((tm, np.degrees(wrap(yaw - course)),
                     gt[j] - gt[i], chord))

    if len(rows) < 10:
        return None
    a = np.array(rows)
    tm = a[:, 0]
    diff = np.degrees(np.unwrap(np.radians(a[:, 1])))
    slope, off = np.polyfit(tm, diff, 1)
    resid = diff - (slope * tm + off)
    return {
        "n": len(a),
        "rms": float(np.sqrt((resid ** 2).mean())),
        "span": float(a[:, 2].mean()),      # 기선을 만드는 데 걸린 시간
        "chord": float(a[:, 3].mean()),
        "slope": float(slope * 60),
    }


def main(path, beta_correct=True):
    yt, yv, gt, lat, lon, st, vx, vy = read(path)
    yaw_u = np.unwrap(yv)
    lat0, lon0 = np.radians(lat[0]), np.radians(lon[0])
    E = R_EARTH * (np.radians(lon) - lon0) * np.cos(lat0)
    N = R_EARTH * (np.radians(lat) - lat0)

    print(f"GPS {len(gt)}개, IMU {len(yt)}개, β 보정 "
          f"{'켬' if beta_correct else '끔'}\n")
    print("  기선   관측수   실제기선   소요시간   지연    잔차 RMS   기울기")
    print("  " + "─" * 66)

    res = []
    for L in BASELINES:
        r = evaluate(L, gt, E, N, yt, yaw_u, st, vx, vy, beta_correct)
        if r is None:
            print(f"  {L:4.1f} m   표본 부족")
            continue
        delay = r["span"] / 2          # 기선 중점이 곧 유효 시각
        print(f"  {L:4.1f} m   {r['n']:5d}   {r['chord']:6.2f} m"
              f"   {r['span']:6.2f} s   {delay:5.2f} s"
              f"   {r['rms']:7.2f}°   {r['slope']:+6.2f}°/분")
        res.append((L, r, delay))

    if len(res) < 2:
        return

    print("\n── 해석 ──")
    ref = [r for r in res if abs(r[0] - 5.0) < 1e-6]
    if ref:
        r5 = ref[0][1]["rms"]
        print(f"5 m 기준 {r5:.2f}° 대비")
        for L, r, d in res:
            ratio = r["rms"] / r5
            ideal = 5.0 / L          # 이론상 1/L
            print(f"  {L:4.1f} m  실측 {ratio:5.2f}배"
                  f"   이론(1/L) {ideal:5.2f}배"
                  f"   {'이론보다 좋음' if ratio < ideal * 0.9 else ('이론과 비슷' if ratio < ideal * 1.1 else '이론보다 나쁨')}")

    print("\n실시간 노드는 미래 점을 못 쓰므로 '지연'만큼 늦은 heading 이 나온다.")
    print("현재 속도 기준이며, 빨라지면 같은 기선에서 지연이 줄어든다.")

    # 관측을 모았을 때의 실효 정확도
    print("\n── 30초간 관측을 평균 냈을 때 ──")
    print("  기선   30초 관측수   실효 오차")
    total_t = gt[-1] - gt[0]
    for L, r, d in res:
        rate = r["n"] / total_t        # 초당 관측 수
        n30 = rate * 30
        if n30 < 1:
            continue
        eff = r["rms"] / np.sqrt(n30)
        print(f"  {L:4.1f} m   {n30:8.1f}개   {eff:6.2f}°")
    print("\n주의: 관측이 서로 독립이라는 가정이다. 실제로는 겹치는 기선이")
    print("      많아 독립이 아니므로 이 값보다 나쁘다. 상한으로 볼 것.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         "/home/hyo/fastlio_ws/go2_outdoor_0731_1114",
         beta_correct="--nobeta" not in sys.argv)
