#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
foot_field_probe.py — leg_odom_refine.py 2단(미끄럼 배제)에 쓸 발 필드를
bag 으로 먼저 검증하는 오프라인 스크립트. **rclpy 노드가 아니다.**

왜 필요한가
-----------
`foot_speed_body`(SportModeState)는 이 저장소 전체에서 한 번도 쓰인 적이
없다. 단위·프레임·부호·갱신주기가 전부 미확인이다. 또한 LowState 는
`foot_force`(발바닥 센서 원시값으로 추정)와 `foot_force_est`(관절 토크
기반 추정값으로 추정) 두 필드를 모두 갖고 있는데(확인:
~/unitree_ros2/example/src/src/read_low_state.cpp:77-89), 어느 쪽이 접지
판정에 적합한지도 확증되지 않았다. leg_odom_refine.py 의 `enable_slip` 을
켜기 전에 이 스크립트로 먼저 검증한다.

판정 기준 (L1 라이다 가속도계를 기각할 때 쓴 것과 동일)
--------------------------------------------------------
    rho = corr( 발 속도로 계산한 몸통 속도,  /utlidar/robot_odom twist )

    rho >= 0.9   2단 진행 가능
    rho <  0.9   2단 폐기, enable_slip 은 영구 False 로 두고 이유를 남길 것
                 (그때 L1 가속도계는 rho=0.19 로 기각했다 — go2_calib.py 참고)

사용
----
    python3 tools/foot_field_probe.py [bag_dir] [--nbins N]

    bag_dir 기본값: ~/data/bags/outdoor/0812/go2_loop1_0812_1449
    (scale_vs_speed.py 와 동일한 기본 bag)

이 스크립트는 sqlite3 로 bag 을 직접 읽는다(scale_vs_speed.py:17-32 의
load() 패턴을 그대로 따름). rclpy 노드를 띄우지 않으므로 bag play 없이
바로 실행할 수 있다.
"""
import glob
import os
import sys
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SMS = get_message('unitree_go/msg/SportModeState')
LOW = get_message('unitree_go/msg/LowState')
ODOM = get_message('nav_msgs/msg/Odometry')

STILL_SPEED_TH = 0.05     # [m/s] leg_odom_refine.py still_speed 기본값과 동일


# ==========================================================================
# bag 읽기 (scale_vs_speed.py:17-32 방식)
# ==========================================================================
def load(d):
    con = sqlite3.connect(glob.glob(os.path.join(d, "*.db3"))[0])
    cur = con.cursor()

    def grab(t):
        r = cur.execute("SELECT id FROM topics WHERE name=?", (t,)).fetchone()
        if not r:
            return []
        return cur.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (r[0],)).fetchall()

    sms_rows = [(ts / 1e9, deserialize_message(x, SMS)) for ts, x in grab('/sportmodestate')]
    low_rows = [(ts / 1e9, deserialize_message(x, LOW)) for ts, x in grab('/lowstate')]
    odom_rows = [(ts / 1e9, deserialize_message(x, ODOM)) for ts, x in grab('/utlidar/robot_odom')]
    con.close()

    sms = {
        't': np.array([t for t, _ in sms_rows]),
        'fpos': np.array([m.foot_position_body for _, m in sms_rows]),   # (N,12)
        'fspd': np.array([m.foot_speed_body for _, m in sms_rows]),      # (N,12)
    } if sms_rows else None

    low = {
        't': np.array([t for t, _ in low_rows]),
        'force': np.array([m.foot_force for _, m in low_rows], dtype=float),       # (N,4)
        'force_est': np.array([m.foot_force_est for _, m in low_rows], dtype=float),  # (N,4)
        'gyro': np.array([m.imu_state.gyroscope for _, m in low_rows]),            # (N,3)
    } if low_rows else None

    odom = {
        't': np.array([t for t, _ in odom_rows]),
        'vx': np.array([m.twist.twist.linear.x for _, m in odom_rows]),
        'vy': np.array([m.twist.twist.linear.y for _, m in odom_rows]),
    } if odom_rows else None

    return sms, low, odom


# ==========================================================================
# 통계 헬퍼
# ==========================================================================
def stats_line(name, v):
    return (f"    {name:<10} mean={v.mean():+8.3f}  std={v.std():7.3f}  "
            f"range=[{v.min():+8.3f}, {v.max():+8.3f}]")


def print_foot_xyz_stats(title, arr12):
    """arr12: (N,12) -> 4발 x 3축."""
    print(f"\n{title}")
    a = arr12.reshape(-1, 4, 3)
    axes = ['x', 'y', 'z']
    for foot in range(4):
        print(f"  발 {foot}:")
        for ax in range(3):
            print(stats_line(axes[ax], a[:, foot, ax]))


def histogram_text(v, nbins=20, width=40):
    hist, edges = np.histogram(v, bins=nbins)
    hmax = hist.max() if hist.max() > 0 else 1
    lines = []
    for i in range(nbins):
        bar = '#' * int(width * hist[i] / hmax)
        lines.append(f"    [{edges[i]:+9.2f}, {edges[i+1]:+9.2f})  {hist[i]:6d}  {bar}")
    return '\n'.join(lines)


def otsu(values, nbins=64):
    """Otsu 이진분리: (임계값, 분리점수[0~1]). 점수가 높을수록 쌍봉이 뚜렷."""
    hist, edges = np.histogram(values, bins=nbins)
    hist = hist.astype(float)
    centers = (edges[:-1] + edges[1:]) / 2
    total = hist.sum()
    if total == 0:
        return float(values.mean()), 0.0

    sum_all = float(np.sum(hist * centers))
    total_var = float(np.var(values))
    if total_var <= 0:
        return float(values.mean()), 0.0

    wB = 0.0
    sumB = 0.0
    best_var = -1.0
    best_th = centers[0]
    for i in range(nbins):
        wB += hist[i]
        if wB == 0:
            continue
        wF = total - wB
        if wF == 0:
            break
        sumB += centers[i] * hist[i]
        mB = sumB / wB
        mF = (sum_all - sumB) / wF
        var_between = wB * wF * (mB - mF) ** 2 / (total ** 2)
        if var_between > best_var:
            best_var = var_between
            best_th = centers[i]

    score = best_var / total_var if total_var > 0 else 0.0
    return float(best_th), float(min(1.0, score))


def corr(a, b):
    a = np.asarray(a).ravel()
    b = np.asarray(b).ravel()
    if len(a) < 3 or np.std(a) == 0 or np.std(b) == 0:
        return float('nan')
    return float(np.corrcoef(a, b)[0, 1])


# ==========================================================================
# 메인 분석
# ==========================================================================
def analyze(d):
    print(f"\n{'='*70}\n{os.path.basename(d)}\n{'='*70}")
    sms, low, odom = load(d)

    if sms is None:
        print("/sportmodestate 없음 — 이 bag 으로는 발 필드 검증 불가")
        return
    if low is None:
        print("/lowstate 없음 — force 필드 검증 불가")
        return
    if odom is None:
        print("/utlidar/robot_odom 없음 — 상관계수 판정 불가")
        return

    print(f"메시지 수: sportmodestate={len(sms['t'])}  lowstate={len(low['t'])}  "
          f"robot_odom={len(odom['t'])}")

    # ---- 1. foot_position_body / foot_speed_body 통계 --------------------
    print_foot_xyz_stats("[1] foot_position_body (발 4개 x xyz)", sms['fpos'])
    print_foot_xyz_stats("[1] foot_speed_body (발 4개 x xyz)", sms['fspd'])

    # ---- 2. foot_force / foot_force_est: 범위·히스토그램·정지/보행 분리 ----
    odom_speed = np.hypot(odom['vx'], odom['vy'])
    low_speed = np.interp(low['t'], odom['t'], odom_speed)
    still_mask = low_speed < STILL_SPEED_TH
    walk_mask = ~still_mask

    for field_name, field in [('foot_force', low['force']), ('foot_force_est', low['force_est'])]:
        flat = field.ravel()
        print(f"\n[2] {field_name}  범위=[{flat.min():.1f}, {flat.max():.1f}]  "
              f"평균={flat.mean():.1f}  표준편차={flat.std():.1f}")
        print(f"  히스토그램 (4발 합쳐서, {len(flat)}개):")
        print(histogram_text(flat))
        for foot in range(4):
            v_still = field[still_mask, foot]
            v_walk = field[walk_mask, foot]
            s_txt = (f"{v_still.mean():+7.1f}±{v_still.std():5.1f}" if len(v_still)
                     else "     (없음)")
            w_txt = (f"{v_walk.mean():+7.1f}±{v_walk.std():5.1f}" if len(v_walk)
                     else "     (없음)")
            print(f"    발 {foot}:  정지구간 {s_txt}   보행구간 {w_txt}")

    # ---- 3. foot_position_body 수치미분 vs foot_speed_body 상관계수 -------
    dt = np.diff(sms['t'])
    dt_safe = np.where(dt > 1e-6, dt, np.nan)
    dpos = np.diff(sms['fpos'], axis=0) / dt_safe[:, None]     # (N-1, 12)
    fspd_aligned = sms['fspd'][1:]                              # (N-1, 12)
    valid = ~np.isnan(dpos).any(axis=1)
    rho_diff = corr(dpos[valid], fspd_aligned[valid])
    print(f"\n[3] foot_position_body 수치미분 vs foot_speed_body 상관계수(전체 flatten): "
          f"rho={rho_diff:.3f}")
    axes = ['x', 'y', 'z']
    dpos_r = dpos.reshape(-1, 4, 3)
    fspd_r = fspd_aligned.reshape(-1, 4, 3)
    for foot in range(4):
        for ax in range(3):
            r = corr(dpos_r[valid, foot, ax], fspd_r[valid, foot, ax])
            print(f"    발 {foot} {axes[ax]}: rho={r:.3f}")
    if not (rho_diff == rho_diff) or rho_diff < 0.9:
        print("  ** rho < 0.9 (또는 계산 불가) — foot_speed_body 를 신뢰하지 마라. **")

    # ---- 4. foot_force vs foot_force_est 상관계수 --------------------------
    rho_ff = corr(low['force'], low['force_est'])
    print(f"\n[4] foot_force vs foot_force_est 상관계수: rho={rho_ff:.3f}")

    # ---- 5. Otsu 이진분리 — 접지판정에 더 나은 필드/임계값 추천 -------------
    th_ff, score_ff = otsu(low['force'].ravel())
    th_est, score_est = otsu(low['force_est'].ravel())
    print(f"\n[5] Otsu 이진분리 점수(쌍봉 뚜렷할수록 1에 가까움):")
    print(f"    foot_force      score={score_ff:.3f}  추천 임계값={th_ff:.1f}")
    print(f"    foot_force_est  score={score_est:.3f}  추천 임계값={th_est:.1f}")
    if score_ff >= score_est:
        print(f"  -> 권장: force_field=foot_force      force_th≈{th_ff:.1f}")
    else:
        print(f"  -> 권장: force_field=foot_force_est  force_th≈{th_est:.1f}")

    # ---- 6. 핵심 판정: 발 속도로 계산한 몸통 속도 vs robot_odom twist -----
    #   v_i = -(fdot_i + omega x f_i),  4발 평균(접지 여부와 무관하게 전부 사용)
    low_gyro_x = np.interp(sms['t'], low['t'], low['gyro'][:, 0])
    low_gyro_y = np.interp(sms['t'], low['t'], low['gyro'][:, 1])
    low_gyro_z = np.interp(sms['t'], low['t'], low['gyro'][:, 2])
    omega = np.stack([low_gyro_x, low_gyro_y, low_gyro_z], axis=1)   # (N,3)

    fpos_r = sms['fpos'].reshape(-1, 4, 3)
    fspd_r_all = sms['fspd'].reshape(-1, 4, 3)
    v_feet = -(fspd_r_all + np.cross(omega[:, None, :], fpos_r))     # (N,4,3)
    v_body_est = v_feet.mean(axis=1)                                  # (N,3) 4발 평균

    odom_vx_i = np.interp(sms['t'], odom['t'], odom['vx'])
    odom_vy_i = np.interp(sms['t'], odom['t'], odom['vy'])

    rho_x = corr(v_body_est[:, 0], odom_vx_i)
    rho_y = corr(v_body_est[:, 1], odom_vy_i)
    rho_xy = corr(np.concatenate([v_body_est[:, 0], v_body_est[:, 1]]),
                  np.concatenate([odom_vx_i, odom_vy_i]))

    print(f"\n[6] 핵심 판정 — 발 속도 기반 몸통속도 vs /utlidar/robot_odom twist")
    print(f"    rho_x={rho_x:.3f}  rho_y={rho_y:.3f}  rho_xy(전체)={rho_xy:.3f}")
    if rho_xy == rho_xy and rho_xy >= 0.9:
        print(f"  -> rho={rho_xy:.3f} >= 0.9 : 2단(미끄럼 배제) 진행 가능. "
              "그래도 enable_slip 은 이 결과를 코드에 반영하기 전까지 False 로 둘 것.")
    else:
        print(f"  -> rho={rho_xy:.3f} < 0.9 : 2단 폐기 권장. enable_slip 을 영구 "
              "False 로 두고, foot_speed_body/foot_force 를 신뢰할 수 없다는 "
              "이유를 leg_odom_refine.py 주석에 남길 것 "
              "(L1 가속도계 기각 기준 rho=0.19 와 같은 원칙).")


def main():
    argv = [a for a in sys.argv[1:] if not a.startswith('--')]
    d = os.path.expanduser(argv[0]) if argv else os.path.expanduser(
        "~/data/bags/outdoor/0812/go2_loop1_0812_1449")
    if not os.path.isdir(d):
        print(f"bag 디렉토리를 찾을 수 없습니다: {d}")
        sys.exit(1)
    analyze(d)


if __name__ == '__main__':
    main()
