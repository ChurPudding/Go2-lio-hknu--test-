#!/usr/bin/env python3
"""
lever_check.py -- LEVER(외부 평행이동) 보정 전후 궤적 지표 비교

배경
----
/aft_mapped_to_init 은 LiDAR 원점의 궤적이다. 그런데 참값으로 쓰는
/utlidar/robot_odom 은 몸통(base_link) 원점의 궤적이다. LiDAR 가 몸통에서
0.322 m 앞에 있으므로, 로봇이 제자리 회전만 해도 LiDAR 원점은 반지름
0.322 m 의 원호를 그린다. 이 호가 그대로 총거리에 더해진다.

    delta_s ~= r * sum|d_theta| = 0.322 * 39.81 rad = 12.8 m

누적 절대 yaw 2281deg 는 진단정리 2절의 기준선 검증값이다.
32.5 + 12.8 = 45.3 m 로, 관측된 45~47 m 평탄값과 맞는다.

보정식 (진단정리 3.5절)
    p_base(map) = p_lidar(map) - R_ML @ R_LB @ r

    R_ML : 오도메트리 쿼터니언 (map <- LiDAR)
    R_LB : base_link -> LiDAR 회전   (go2_calib)
    r    : base_link 원점 -> LiDAR 원점, base_link 프레임 표현 (= LEVER)

사용법
    python3 lever_check.py ~/fastlio_ws/imufix2.csv
    python3 lever_check.py ~/fastlio_ws/imufix2.csv --hz 20

입력 CSV 헤더: t,x,y,z,qx,qy,qz,qw
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from go2_calib import R_LB, LEVER
except ImportError:
    sys.stderr.write('go2_calib.py 를 이 스크립트와 같은 디렉터리에 두십시오.\n')
    raise

TRUE_PATH = 32.5        # robot_odom 수평 총거리 [m]
TRUE_LOOP = 0.556       # robot_odom 루프 클로저 [m]
TRUE_YAW_ABS = 2281.0   # robot_odom 누적 절대 yaw [deg]


def quat_to_R(q):
    """q = (qx,qy,qz,qw) -> 3x3 회전행렬. 입력 배열 (N,4) 를 받아 (N,3,3)."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    N = len(q)
    R = np.empty((N, 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def load(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    hdr = [h.strip().lower() for h in rows[0]]
    need = ['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    if all(n in hdr for n in need):
        idx = [hdr.index(n) for n in need]
        body = rows[1:]
    else:
        sys.stderr.write('헤더를 못 찾아 위치 기준(t,x,y,z,qx,qy,qz,qw)으로 읽습니다.\n')
        idx = list(range(8))
        body = rows[1:] if not rows[0][0].lstrip('-').replace('.', '').isdigit() else rows
    d = np.array([[float(r[i]) for i in idx] for r in body if len(r) >= 8])
    t = d[:, 0]
    if t[0] > 1e12:          # ns 로 저장된 경우
        t = t / 1e9
    return t, d[:, 1:4], d[:, 4:8]


def horiz_path(p, k):
    q = p[::k]
    return np.hypot(np.diff(q[:, 0]), np.diff(q[:, 1])).sum()


def cum_abs_yaw(R):
    """연속 자세 간 상대회전의 연직축 성분을 누적한다."""
    tot = 0.0
    for i in range(1, len(R)):
        dR = R[i - 1].T @ R[i]
        ang = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
        if ang < 1e-9:
            continue
        # 회전축 (dR - dR.T) 에서 추출
        ax = np.array([dR[2, 1] - dR[1, 2],
                       dR[0, 2] - dR[2, 0],
                       dR[1, 0] - dR[0, 1]]) / (2 * np.sin(ang))
        tot += abs(ang * ax[2])          # map 프레임 z = 중력 정렬축
    return np.degrees(tot)


def summarize(name, t, p, rate):
    print('\n--- %s ---' % name)
    print('%9s %8s %14s %9s' % ('실효Hz', 'k', '수평총거리m', '표본수'))
    seen = set()
    for hz in (rate, 200, 100, 50, 20, 15, 10, 5, 2):
        k = max(1, int(round(rate / hz)))
        if k in seen:
            continue
        seen.add(k)
        n = len(p[::k])
        print('%9.1f %8d %14.2f %9d' % (rate / k, k, horiz_path(p, k), n))
    loop = np.hypot(p[-1, 0] - p[0, 0], p[-1, 1] - p[0, 1])
    print('루프 클로저 = %.3f m   (참값 %.3f)' % (loop, TRUE_LOOP))
    print('z 범위 %.4f ~ %.4f m   (p-p %.1f mm)'
          % (p[:, 2].min(), p[:, 2].max(),
             1000 * (p[:, 2].max() - p[:, 2].min())))
    print('시작->끝 z = %+.1f mm' % (1000 * (p[-1, 2] - p[0, 2])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--hz', type=float, default=20.0,
                    help='요약 비교에 쓸 기준 표본율 (기본 20 Hz)')
    a = ap.parse_args()

    t, p_l, q = load(a.csv)
    rate = len(t) / (t[-1] - t[0])
    print('입력 %s' % a.csv)
    print('표본 %d개, %.2f초, %.0f Hz' % (len(t), t[-1] - t[0], rate))

    R_ML = quat_to_R(q)

    # LEVER 를 LiDAR 프레임으로 옮긴 뒤 map 프레임으로 회전
    v_L = R_LB @ LEVER                       # (3,)
    off = R_ML @ v_L                         # (N,3)
    p_b = p_l - off

    print('\nR_LB @ LEVER (LiDAR 프레임) = (%+.4f, %+.4f, %+.4f) m, |r|=%.4f'
          % (v_L[0], v_L[1], v_L[2], np.linalg.norm(v_L)))

    summarize('보정 전: LiDAR 원점 (/aft_mapped_to_init)', t, p_l, rate)
    summarize('보정 후: 몸통 원점 (base_link)', t, p_b, rate)

    # 이론 예측 검증
    k = max(1, int(round(rate / 200)))       # 200 Hz 로 줄여 yaw 누적 (연산량)
    yaw_abs = cum_abs_yaw(R_ML[::k])
    r_h = np.hypot(LEVER[0], LEVER[1])
    pred = r_h * np.radians(yaw_abs)
    kk = max(1, int(round(rate / a.hz)))
    d_l, d_b = horiz_path(p_l, kk), horiz_path(p_b, kk)

    print('\n=== 이론 예측 대조 (%.0f Hz 기준) ===' % (rate / kk))
    print('누적 절대 yaw    = %8.1f deg   (robot_odom 참값 %.0f)'
          % (yaw_abs, TRUE_YAW_ABS))
    print('예측 초과 거리   = %8.2f m     (= %.3f m x %.2f rad)'
          % (pred, r_h, np.radians(yaw_abs)))
    print('실측 초과 거리   = %8.2f m     (보정 전 %.2f - 보정 후 %.2f)'
          % (d_l - d_b, d_l, d_b))
    print('보정 후 vs 참값  = %8.2f m     (참값 %.1f, 오차 %+.1f%%)'
          % (d_b, TRUE_PATH, 100 * (d_b - TRUE_PATH) / TRUE_PATH))

    print('\n판정:')
    err = abs(d_b - TRUE_PATH) / TRUE_PATH
    if err < 0.10:
        print('  통과. 44% 과대는 LEVER 미보정이 원인으로 확정.')
        print('  -> eval_lio.py 에 이 보정을 넣고 기존 결과를 전부 재계산할 것.')
    elif d_l - d_b > 8.0:
        print('  부분 설명. LEVER 보정이 %.1f m 를 설명하지만 %.1f m 가 남는다.'
              % (d_l - d_b, d_b - TRUE_PATH))
        print('  -> 남은 몫은 별도 원인. 보행 진동(2~3Hz) 여부를 스펙트럼으로 볼 것.')
    else:
        print('  기각. LEVER 로는 설명되지 않는다. LEVER 값 또는 R_LB 을 재검토할 것.')


if __name__ == '__main__':
    main()
