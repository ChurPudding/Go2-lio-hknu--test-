#!/usr/bin/env python3
"""GPS 진행방향과 IMU yaw 를 비교해 오프셋과 표류율을 구한다.

원리
----
GPS 두 점의 변위로 진행방향(course over ground)을 구하고, 같은 시각의
IMU yaw 와 비교한다. 차이가 시간에 대해 평평하면 상수 오프셋이고,
기울어지면 그 기울기가 자이로 표류율이다.

GPS 잡음이 2.4 m 이므로 짧은 기선(baseline)은 쓸 수 없다. 최소 변위
MIN_BASELINE 이상 벌어진 점끼리만 짝짓고, 직선·전진 구간만 남긴다.

사용법
------
    source /opt/ros/humble/setup.bash
    source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
    python3 yaw_gps_check.py ~/fastlio_ws/go2_outdoor_0731_1114
"""
import json
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from unitree_go.msg import LowState, SportModeState

LOWSTATE_EVERY = 25   #     500 Hz -> 20 Hz. 디코딩이 느려 솎아낸다
SPORT_EVERY = 15      #     약 300 Hz -> 20 Hz
MIN_BASELINE = 5.0    # m   기선 최소 길이
MAX_SPAN = 25.0       # s   기선 최대 시간
MAX_BOW = 1.0         # m   직선 판정: 현에서 벗어난 최대 거리
MIN_SPEED = 0.20      # m/s 전진 최소 속도
MAX_LATERAL = 0.10    # m/s 게걸음 허용 한도. BETA_CORRECT 시 완화된다
BETA_MAX_LATERAL = 0.40  # m/s β 보정을 켰을 때의 게걸음 한도
R_EARTH = 6378137.0


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=["/lowstate", "/gnss", "/sportmodestate"]))

    yaw_t, yaw_v = [], []
    gps_t, gps_lat, gps_lon, gps_hdop, gps_sat = [], [], [], [], []
    spd_t, spd_vx, spd_vy = [], [], []
    n_bad = 0

    n_low = n_sport = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        t = ts * 1e-9
        if topic == "/lowstate":
            n_low += 1
            if n_low % LOWSTATE_EVERY:
                continue                        # 디코딩 자체를 건너뛴다
            m = deserialize_message(data, LowState)
            q = m.imu_state.quaternion          # [w, x, y, z]
            yaw_t.append(t)
            yaw_v.append(np.arctan2(
                2 * (q[0] * q[3] + q[1] * q[2]),
                1 - 2 * (q[2] ** 2 + q[3] ** 2)))
        elif topic == "/sportmodestate":
            n_sport += 1
            if n_sport % SPORT_EVERY:
                continue
            m = deserialize_message(data, SportModeState)
            spd_t.append(t)
            spd_vx.append(m.velocity[0])
            spd_vy.append(m.velocity[1])
        elif topic == "/gnss":
            from std_msgs.msg import String
            try:
                d = json.loads(deserialize_message(data, String).data)
                if not d.get("fixed"):
                    continue
                gps_t.append(t)
                gps_lat.append(d["latitude"])
                gps_lon.append(d["longitude"])
                gps_hdop.append(d.get("hdop", float("nan")))
                gps_sat.append(d.get("satellite_inuse", 0))
            except Exception:
                n_bad += 1

    return (np.array(yaw_t), np.array(yaw_v),
            np.array(gps_t), np.array(gps_lat), np.array(gps_lon),
            np.array(gps_hdop), np.array(gps_sat),
            np.array(spd_t), np.array(spd_vx), np.array(spd_vy), n_bad)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def main(bag_path, beta_correct=False):
    (yt, yv, gt, lat, lon, hdop, sat, st, vx, vy, n_bad) = read_bag(bag_path)
    t0 = min(yt[0], gt[0], st[0])
    yt, gt, st = yt - t0, gt - t0, st - t0

    yaw_u = np.unwrap(yv)
    netrot = yaw_u - yaw_u[0]     # 부호를 살린 순 회전량. 축척 오차는 여기에만 작용한다

    print(f"IMU  {len(yt)}개  {len(yt)/ (yt[-1]-yt[0]):.1f} Hz")
    print(f"GPS  {len(gt)}개  {len(gt)/ (gt[-1]-gt[0]):.2f} Hz"
          f"  파싱실패 {n_bad}")
    print(f"     위성 {sat.min()}~{sat.max()} (평균 {sat.mean():.1f})"
          f"  HDOP {np.nanmin(hdop):.2f}~{np.nanmax(hdop):.2f}"
          f" (평균 {np.nanmean(hdop):.2f})")

    # 국소 ENU 변환
    lat0, lon0 = np.radians(lat[0]), np.radians(lon[0])
    E = R_EARTH * (np.radians(lon) - lon0) * np.cos(lat0)
    N = R_EARTH * (np.radians(lat) - lat0)
    step = np.hypot(np.diff(E), np.diff(N))
    print(f"     이동거리 누계 {step.sum():.1f} m,"
          f" 중복좌표 {int((step < 1e-6).sum())}개")
    print()

    # 기선 후보 만들기
    rows = []
    reject = {"기선부족": 0, "곡선": 0, "저속": 0, "게걸음": 0}
    for i in range(len(gt)):
        j = i + 1
        while j < len(gt):
            d = np.hypot(E[j] - E[i], N[j] - N[i])
            if gt[j] - gt[i] > MAX_SPAN:
                j = -1
                break
            if d >= MIN_BASELINE:
                break
            j += 1
        if j < 0 or j >= len(gt):
            reject["기선부족"] += 1
            continue

        # 직선성 — 중간 점들이 현에서 얼마나 벗어나는가
        dE, dN = E[j] - E[i], N[j] - N[i]
        L = np.hypot(dE, dN)
        mid = slice(i + 1, j)
        if j - i > 1:
            bow = np.abs((E[mid] - E[i]) * dN - (N[mid] - N[i]) * dE) / L
            if bow.max() > MAX_BOW:
                reject["곡선"] += 1
                continue

        # 속도 조건
        sel = (st >= gt[i]) & (st <= gt[j])
        if sel.sum() < 5:
            continue
        mvx, mvy = vx[sel].mean(), np.abs(vy[sel]).mean()
        if mvx < MIN_SPEED:
            reject["저속"] += 1
            continue
        lat_limit = BETA_MAX_LATERAL if beta_correct else MAX_LATERAL
        if mvy > lat_limit:
            reject["게걸음"] += 1
            continue

        # 측면미끄러짐각 — 몸통이 가리키는 방향과 실제 이동 방향의 차이
        beta = np.arctan2(vy[sel].mean(), vx[sel].mean())

        tm = 0.5 * (gt[i] + gt[j])
        course = np.arctan2(dN, dE)            # ENU: 동쪽 기준 반시계
        if beta_correct:
            course = course - beta             # 이동 방향 -> 몸통 방향
        yaw = np.interp(tm, yt, yaw_u)
        cum = np.interp(tm, yt, netrot)
        rows.append((tm, np.degrees(course), np.degrees(wrap(yaw)),
                     np.degrees(wrap(yaw - course)), L, mvx,
                     np.degrees(cum), np.degrees(beta)))

    print(f"기선 채택 {len(rows)}개  " +
          "  ".join(f"{k} {v}" for k, v in reject.items()))
    if len(rows) < 10:
        print("\n표본이 너무 적다. MIN_BASELINE 을 줄이거나 조건을 완화할 것.")
        return

    a = np.array(rows)
    tm, diff = a[:, 0], np.unwrap(np.radians(a[:, 3]))
    diff = np.degrees(diff)

    print()
    print("  시각      GPS방위    IMU yaw     차이    기선   속도")
    for r in a[:: max(1, len(a) // 25)]:
        print(f"{r[0]:7.1f} s {r[1]:8.1f}° {r[2]:9.1f}° {r[3]:8.1f}°"
              f" {r[4]:6.1f} m {r[5]:5.2f}")

    slope, offset = np.polyfit(tm, diff, 1)
    resid = diff - (slope * tm + offset)
    print()
    print(f"차이 평균   {diff.mean():+8.2f}°   표준편차 {diff.std():.2f}°")
    print(f"선형적합    오프셋 {offset:+.2f}°,"
          f" 기울기 {slope*60:+.3f}°/분")
    print(f"적합 잔차   RMS {np.sqrt((resid**2).mean()):.2f}°")
    print()
    span_min = (tm[-1] - tm[0]) / 60
    total = slope * 60 * span_min
    print(f"{span_min:.1f}분 동안 누적 heading 변화 {total:+.2f}°")

    # 시간 표류 모델 vs 회전 스케일 모델 — 어느 쪽이 데이터를 더 잘 설명하는가
    cr = a[:, 6]
    s_rot, o_rot = np.polyfit(cr, diff, 1)
    r_rot = diff - (s_rot * cr + o_rot)
    rms_rot = np.sqrt((r_rot ** 2).mean())
    rms_time = np.sqrt((resid ** 2).mean())

    # 두 모델을 동시에 넣어 각각의 고유 기여를 본다
    A = np.column_stack([tm, cr, np.ones_like(tm)])
    coef, *_ = np.linalg.lstsq(A, diff, rcond=None)
    r_both = diff - A @ coef
    rms_both = np.sqrt((r_both ** 2).mean())

    print()
    print("── 모델 비교 ──")
    print(f"시간 표류만   기울기 {slope*60:+.3f}°/분      잔차 RMS {rms_time:.2f}°")
    print(f"회전 스케일만 기울기 {s_rot*100:+.3f}%        잔차 RMS {rms_rot:.2f}°")
    print(f"둘 다         {coef[0]*60:+.3f}°/분 + {coef[1]*100:+.3f}%"
          f"  잔차 RMS {rms_both:.2f}°")
    print(f"순 회전량 {cr[-1]-cr[0]:+.1f}°,  경과 {span_min:.1f}분"
          f",  두 변수 상관 {np.corrcoef(tm, cr)[0,1]:+.3f}")
    if rms_rot < rms_time * 0.95:
        print("→ 회전 스케일 오차가 우세하다. 자이로 z 축척 보정이 효과적이다.")
    elif rms_time < rms_rot * 0.95:
        print("→ 시간 표류가 우세하다. 축척 보정으로는 해결되지 않는다.")
    else:
        print("→ 두 모델이 구분되지 않는다. 상관이 높아 이 bag 으로는 판정 불가.")

    # 직선 구간별 분해 — 순 회전이 멈춘 동안 차이가 움직이는지 본다
    print()
    print("── 직선 구간별 ──")
    turn = np.abs(np.gradient(cr, tm))          # 순 회전 변화율 (rad/s 아님, deg/s)
    straight = turn < np.degrees(0.05)
    segs, s0 = [], None
    for k, v in enumerate(straight):
        if v and s0 is None:
            s0 = k
        elif not v and s0 is not None:
            if tm[k - 1] - tm[s0] > 15:
                segs.append((s0, k - 1))
            s0 = None
    if s0 is not None and tm[-1] - tm[s0] > 15:
        segs.append((s0, len(tm) - 1))

    if len(segs) < 2:
        print("직선 구간을 2개 이상 찾지 못했다.")
    else:
        print("  구간          yaw       차이 평균   내부 기울기")
        for p, q in segs:
            sl, _ = np.polyfit(tm[p:q + 1], diff[p:q + 1], 1)
            print(f"{tm[p]:6.0f}~{tm[q]:5.0f} s  {a[p,2]:6.1f}°"
                  f"  {diff[p:q+1].mean():8.2f}°  {sl*60:+8.2f}°/분")
        inner = np.mean([np.polyfit(tm[p:q + 1], diff[p:q + 1], 1)[0]
                         for p, q in segs]) * 60
        print(f"\n직선 내부 평균 기울기 {inner:+.2f}°/분"
              f"  vs  전체 기울기 {slope*60:+.2f}°/분")
        if abs(inner) < abs(slope * 60) * 0.4:
            print("→ 직선에서는 안 벌어진다. 회전할 때만 생기는 축척 오차다.")
        else:
            print("→ 직선에서도 벌어진다. 시간 표류 성분이 있다.")

    print(f"\n── β (측면미끄러짐각) ──")
    bt = a[:, 7]
    print(f"평균 {bt.mean():+.2f}°   표준편차 {bt.std():.2f}°"
          f"   범위 {bt.min():+.2f} ~ {bt.max():+.2f}°")
    se = bt.std() / np.sqrt(len(bt))
    if abs(bt.mean()) > 2 * se:
        print(f"→ 평균이 0 과 유의하게 다르다 (표준오차 {se:.2f}°)."
              " 계통 편향이 존재한다.")
    else:
        print(f"→ 평균이 0 과 구분되지 않는다 (표준오차 {se:.2f}°). 잡음에 가깝다.")
    print(f"보정 적용: {'예' if beta_correct else '아니오'}"
          f"   (게걸음 한도 {BETA_MAX_LATERAL if beta_correct else MAX_LATERAL} m/s)")

    np.savetxt("results/yaw_gps_diff.csv", a, delimiter=",",
               header="t,course_deg,yaw_deg,diff_deg,baseline_m,vx,netrot_deg,beta_deg",
               comments="")
    print("\nresults/yaw_gps_diff.csv 저장")


if __name__ == "__main__":
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    main(args[0] if args else "/home/hyo/fastlio_ws/go2_outdoor_0731_1114",
         beta_correct="--beta" in sys.argv)
