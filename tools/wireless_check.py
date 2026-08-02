#!/usr/bin/env python3
"""리모컨 입력을 분석해 무입력(자율) 구간을 찾는다.

목적
----
"정말 손을 안 댔는가" 를 사후에 증명한다. 자율주행 실험 결과를 해석할 때
조종이 섞였는지 아닌지가 갈리므로 반드시 필요하다.

부수적으로 스틱 축과 실제 로봇 움직임을 대조해 매핑을 확인한다.

사용법
------
    python3 wireless_check.py ~/fastlio_ws/go2_outdoor_all_0731_1128
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from unitree_go.msg import LowState, SportModeState, WirelessController

DEADZONE = 0.02       #     이 아래는 무입력으로 본다
MIN_IDLE = 3.0        # s   이보다 긴 무입력만 구간으로 잡는다
NOTURN_SETTLE = 1.5   # s   회전 명령을 뗀 뒤 이만큼은 관성으로 보고 버린다
NOTURN_MIN = 8.0      # s   이보다 긴 무회전 구간만 본다
SPORT_EVERY = 15


def read(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=["/wirelesscontroller", "/sportmodestate", "/lowstate"]))

    wt, stick, keys = [], [], []
    st, vel = [], []
    yt, yv = [], []
    n_sp = n_low = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        t = ts * 1e-9
        if topic == "/wirelesscontroller":
            m = deserialize_message(data, WirelessController)
            wt.append(t)
            stick.append([m.lx, m.ly, m.rx, m.ry])
            keys.append(m.keys)
        elif topic == "/lowstate":
            n_low += 1
            if n_low % 25:
                continue
            m = deserialize_message(data, LowState)
            q = m.imu_state.quaternion
            yt.append(t)
            yv.append(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
        else:
            n_sp += 1
            if n_sp % SPORT_EVERY:
                continue
            m = deserialize_message(data, SportModeState)
            st.append(t)
            vel.append([m.velocity[0], m.velocity[1], m.yaw_speed])
    return (np.array(wt), np.array(stick), np.array(keys),
            np.array(st), np.array(vel), np.array(yt), np.array(yv))


def main(path):
    wt, stick, keys, st, vel, yt, yv = read(path)
    if len(wt) == 0:
        print("/wirelesscontroller 가 이 bag 에 없다.")
        print("앞으로 녹화 토픽 목록에 반드시 넣을 것.")
        return

    t0 = min(wt[0], st[0]) if len(st) else wt[0]
    wt, st = wt - t0, st - t0
    if len(yt):
        yt = yt - t0
        yaw_u = np.unwrap(yv)

    print(f"리모컨 {len(wt)}개  {len(wt)/(wt[-1]-wt[0]):.1f} Hz"
          f"  기록 {wt[-1]:.1f} s\n")

    names = ["lx", "ly", "rx", "ry"]
    print("축별 사용 현황")
    for k, n in enumerate(names):
        v = stick[:, k]
        act = np.abs(v) > DEADZONE
        print(f"  {n}  범위 {v.min():+.3f} ~ {v.max():+.3f}"
              f"   입력 있던 시간 {act.mean()*100:5.1f} %")

    nz = keys != 0
    print(f"  keys  0 이 아닌 샘플 {nz.sum()}개 ({nz.mean()*100:.1f} %)")
    if nz.any():
        print(f"        나타난 값: {sorted(set(keys[nz].tolist()))[:10]}")

    # 무입력 구간 찾기
    active = (np.abs(stick) > DEADZONE).any(axis=1) | (keys != 0)
    idle, s = [], None
    for k, a in enumerate(active):
        if not a and s is None:
            s = k
        elif a and s is not None:
            if wt[k - 1] - wt[s] >= MIN_IDLE:
                idle.append((s, k - 1))
            s = None
    if s is not None and wt[-1] - wt[s] >= MIN_IDLE:
        idle.append((s, len(wt) - 1))

    total = sum(wt[b] - wt[a] for a, b in idle)
    print(f"\n무입력 구간 ({MIN_IDLE:.0f} s 이상) {len(idle)}개,"
          f" 합계 {total:.1f} s / {wt[-1]:.1f} s ({total/wt[-1]*100:.1f} %)")

    if idle:
        print("\n  구간              길이   그동안 로봇은")
        for a, b in idle:
            dur = wt[b] - wt[a]
            if len(st):
                sel = (st >= wt[a]) & (st <= wt[b])
                if sel.sum():
                    v = vel[sel]
                    desc = (f"vx {v[:,0].mean():+.2f}  vy {v[:,1].mean():+.2f}"
                            f"  yaw_speed {np.degrees(v[:,2].mean()):+6.1f}°/s")
                else:
                    desc = "-"
            else:
                desc = "-"
            print(f"  {wt[a]:6.1f}~{wt[b]:6.1f} s  {dur:6.1f} s   {desc}")

        print("\n무입력인데 vx 나 yaw_speed 가 0 이 아니면 관성이거나"
              " 로봇이 스스로 움직인 것이다.")
    else:
        print("\n무입력 구간이 없다. 이 bag 은 전 구간 조종 주행이다.")

    # 스틱 축 매핑 확인
    if len(st) > 10:
        print("\n축 ↔ 움직임 상관 (매핑 확인용)")
        vi = np.column_stack([
            np.interp(wt, st, vel[:, 0]),
            np.interp(wt, st, vel[:, 1]),
            np.interp(wt, st, vel[:, 2]),
        ])
        print("         vx      vy   yaw_speed")
        for k, n in enumerate(names):
            if np.std(stick[:, k]) < 1e-6:
                print(f"  {n}    (입력 없음)")
                continue
            r = [np.corrcoef(stick[:, k], vi[:, j])[0, 1] for j in range(3)]
            print(f"  {n}  {r[0]:+6.2f}  {r[1]:+6.2f}  {r[2]:+6.2f}")
        print("\n절댓값이 큰 쪽이 그 축의 역할이다.")

    # ── 회전 명령이 없던 구간의 방향 변화 ──────────────────────────
    if len(yt) == 0:
        print("\n/lowstate 가 없어 무회전 구간 분석을 건너뛴다.")
        return

    print("\n" + "─" * 60)
    print("회전 명령(rx)이 없던 구간에서 로봇이 스스로 얼마나 돌았는가")
    print("─" * 60)

    turning = np.abs(stick[:, 2]) > DEADZONE
    segs, s = [], None
    for k, a in enumerate(turning):
        if not a and s is None:
            s = k
        elif a and s is not None:
            segs.append((s, k - 1))
            s = None
    if s is not None:
        segs.append((s, len(wt) - 1))

    rows = []
    print("\n  구간              길이   yaw 변화    변화율     평균 vx")
    for a, b in segs:
        t_start = wt[a] + NOTURN_SETTLE       # 관성 구간 버리기
        t_end = wt[b]
        if t_end - t_start < NOTURN_MIN:
            continue
        y0 = np.interp(t_start, yt, yaw_u)
        y1 = np.interp(t_end, yt, yaw_u)
        dur = t_end - t_start
        d = np.degrees(y1 - y0)
        if len(st):
            sel = (st >= t_start) & (st <= t_end)
            mvx = vel[sel, 0].mean() if sel.sum() else float("nan")
        else:
            mvx = float("nan")
        print(f"  {t_start:6.1f}~{t_end:6.1f} s  {dur:6.1f} s"
              f" {d:+8.2f}° {d/dur*60:+8.2f}°/분  {mvx:6.2f} m/s")
        rows.append((dur, d, mvx))

    if not rows:
        print(f"  {NOTURN_MIN:.0f} s 이상인 무회전 구간이 없다.")
        return

    r = np.array(rows)
    w = r[:, 0]
    rate = np.average(r[:, 1] / r[:, 0] * 60, weights=w)
    print(f"\n무회전 구간 {len(rows)}개, 합계 {w.sum():.1f} s"
          f"  ({w.sum()/wt[-1]*100:.0f} % of bag)")
    print(f"가중 평균 방향 변화율  {rate:+.2f}°/분")
    print(f"총 방향 변화 {r[:,1].sum():+.2f}°")

    print()
    if abs(rate) < 1.0:
        print("→ 회전 명령이 없으면 거의 안 돈다. 직진성은 양호하다.")
        print("  D→B 현상은 IMU 가 인식 못 한 회전(=IMU 오차)이거나")
        print("  게걸음 누적 쪽을 의심해야 한다.")
    else:
        print("→ 회전 명령이 없어도 로봇이 스스로 돈다.")
        print(f"  이 비율이면 2분 직진에 {abs(rate)*2:.1f}° 어긋난다.")
        print("  D→B 현상과 방향이 맞는지 부호를 확인할 것")
        print("  (+ 가 반시계=왼쪽).")

    print("\n주의: 이것은 IMU 가 인식한 회전량이다. 실제로 돈 것인지")
    print("      IMU 만 그렇게 읽은 것인지는 GPS 와 대조해야 갈린다.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         "/home/hyo/fastlio_ws/go2_outdoor_all_0731_1128")
