#!/usr/bin/env python3
"""
traj_to_csv.py — Point-LIO 궤적 bag(/aft_mapped_to_init)을 CSV로 변환한다.

목적:
  - 녹화가 온전한지 확인 (메시지 수, 값 범위)
  - 궤적이 언제/어디서 발산하는지 분석 (RViz는 발산 시 화면이 날아가 보기 힘듦)
  - MATLAB / 파이썬으로 궤적을 정밀하게 다시 그리기 위한 데이터

사용:
    source /opt/ros/humble/setup.bash
    python3 traj_to_csv.py <bag_db3_경로> <출력_csv>

예:
    python3 traj_to_csv.py \\
        traj_run1_fixafter/traj_run1_fixafter_0.db3 \\
        traj_run1_fixafter.csv

인자를 생략하면 위 예시 경로를 기본값으로 쓴다.

출력 컬럼:
    t_sec  : 시작=0 기준 경과 시간(초)
    x,y,z  : 위치 (m)
    qx,qy,qz,qw : 자세 쿼터니언
    yaw_deg     : 쿼터니언에서 계산한 yaw (도) — 궤적 방향 확인용
    dist_from_origin : 원점(0,0,0)으로부터의 3D 거리 (m) — 발산 탐지용
"""
import sqlite3
import csv
import sys
import math

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def quat_to_yaw_deg(qx, qy, qz, qw):
    """쿼터니언 -> yaw(도). z축 회전만 추출."""
    # yaw = atan2( 2(qw*qz + qx*qy), 1 - 2(qy^2 + qz^2) )
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny, cosy))


def main():
    bag = (sys.argv[1] if len(sys.argv) > 1
           else "traj_run1_fixafter/traj_run1_fixafter_0.db3")
    out = (sys.argv[2] if len(sys.argv) > 2
           else "traj_run1_fixafter.csv")

    con = sqlite3.connect(bag)
    cur = con.cursor()

    # /aft_mapped_to_init 토픽 id 찾기
    row = cur.execute(
        "SELECT id FROM topics WHERE name='/aft_mapped_to_init'"
    ).fetchone()
    if row is None:
        print("오류: bag 안에 /aft_mapped_to_init 토픽이 없습니다.")
        print("담긴 토픽 목록:")
        for (n,) in cur.execute("SELECT name FROM topics"):
            print("  ", n)
        sys.exit(1)
    tid = row[0]

    msgs = cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,)
    ).fetchall()

    if not msgs:
        print("오류: 메시지가 0개입니다. 녹화가 비어 있습니다.")
        sys.exit(1)

    # 발산 통계용
    max_dist = 0.0
    diverge_t = None   # 처음으로 거리 100m를 넘긴 시각

    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sec", "x", "y", "z",
                    "qx", "qy", "qz", "qw",
                    "yaw_deg", "dist_from_origin"])
        t0 = None
        for ts, data in msgs:
            msg = deserialize_message(bytes(data), Odometry)
            if t0 is None:
                t0 = ts
            t = (ts - t0) * 1e-9  # 초, 시작=0

            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            yaw = quat_to_yaw_deg(o.x, o.y, o.z, o.w)
            dist = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)

            if dist > max_dist:
                max_dist = dist
            if diverge_t is None and dist > 100.0:
                diverge_t = t

            w.writerow([f"{t:.4f}",
                        f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
                        f"{o.x:.6f}", f"{o.y:.6f}", f"{o.z:.6f}", f"{o.w:.6f}",
                        f"{yaw:.3f}", f"{dist:.3f}"])

    # 요약 출력
    print(f"완료: {out}")
    print(f"  메시지 수    : {len(msgs)}")
    print(f"  기록 시간    : {(msgs[-1][0]-msgs[0][0])*1e-9:.1f} s")
    print(f"  원점 최대거리: {max_dist:.1f} m")
    if diverge_t is not None:
        print(f"  ** 발산 감지 : t={diverge_t:.1f}s 부터 원점거리 100m 초과 **")
    else:
        print(f"  발산 없음(원점거리 100m 이내 유지)")
    print()
    print("다음: MATLAB이나 파이썬에서 x-y 궤적을 그려 발산 시점을 확인하세요.")


if __name__ == "__main__":
    main()
