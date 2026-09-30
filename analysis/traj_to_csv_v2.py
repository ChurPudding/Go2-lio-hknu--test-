#!/usr/bin/env python3
"""
traj_to_csv_v2.py — Odometry 타입 토픽을 bag에서 CSV로 변환한다.

이전 버전과 차이: 세 번째 인자로 토픽 이름을 받는다.
  - Point-LIO 궤적:  /aft_mapped_to_init
  - 다리 오도메트리:  /utlidar/robot_odom
  둘 다 nav_msgs/msg/Odometry 타입이라 같은 스크립트로 처리된다.

목적:
  - 녹화가 온전한지 확인 (메시지 수, 값 범위)
  - 궤적이 언제/어디서 발산하는지 분석
  - 다리 오도메트리로 "bag이 정상인지 vs 알고리즘이 문제인지" 판별
  - MATLAB/파이썬으로 궤적을 다시 그리기 위한 데이터

사용:
    source /opt/ros/humble/setup.bash
    python3 traj_to_csv_v2.py <bag_db3> <출력_csv> [토픽이름]

예 1) Point-LIO 궤적 (기본 토픽):
    python3 traj_to_csv_v2.py \\
        traj_run2_fixafter/traj_run2_fixafter_0.db3 traj_run2_fixafter.csv

예 2) 원본 bag에서 다리 오도메트리 뽑기:
    python3 traj_to_csv_v2.py \\
        lio_test_bag_loop_run1/lio_test_bag_loop_run1_0.db3 \\
        legodom_run1.csv /utlidar/robot_odom

토픽 인자를 생략하면 /aft_mapped_to_init 를 기본으로 쓴다.

출력 컬럼:
    t_sec, x, y, z, qx, qy, qz, qw, yaw_deg, dist_from_origin
"""
import sqlite3
import csv
import sys
import math

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def quat_to_yaw_deg(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny, cosy))


def main():
    if len(sys.argv) < 2:
        bag = "traj_run1_fixafter/traj_run1_fixafter_0.db3"
    else:
        bag = sys.argv[1]

    out = sys.argv[2] if len(sys.argv) > 2 else "traj.csv"
    topic = sys.argv[3] if len(sys.argv) > 3 else "/aft_mapped_to_init"

    con = sqlite3.connect(bag)
    cur = con.cursor()

    row = cur.execute(
        "SELECT id FROM topics WHERE name=?", (topic,)
    ).fetchone()
    if row is None:
        print(f"오류: bag 안에 '{topic}' 토픽이 없습니다.")
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
        print(f"오류: '{topic}' 메시지가 0개입니다. 녹화가 비어 있습니다.")
        sys.exit(1)

    max_dist = 0.0
    diverge_t = None

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
            t = (ts - t0) * 1e-9

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

    print(f"완료: {out}   (토픽: {topic})")
    print(f"  메시지 수    : {len(msgs)}")
    print(f"  기록 시간    : {(msgs[-1][0]-msgs[0][0])*1e-9:.1f} s")
    print(f"  원점 최대거리: {max_dist:.1f} m")
    if diverge_t is not None:
        print(f"  ** 발산 감지 : t={diverge_t:.1f}s 부터 원점거리 100m 초과 **")
    else:
        print(f"  발산 없음(원점거리 100m 이내 유지) -> 정상 궤적일 가능성")

    # 시작점과 끝점 (폐루프 확인용)
    con2 = sqlite3.connect(bag)
    c2 = con2.cursor()
    first = deserialize_message(bytes(msgs[0][1]), Odometry).pose.pose.position
    last = deserialize_message(bytes(msgs[-1][1]), Odometry).pose.pose.position
    loop_err = math.sqrt((last.x-first.x)**2 + (last.y-first.y)**2)
    print(f"  시작점: ({first.x:.2f}, {first.y:.2f})")
    print(f"  끝점  : ({last.x:.2f}, {last.y:.2f})")
    print(f"  폐루프 오차(시작-끝 xy거리): {loop_err:.3f} m")


if __name__ == "__main__":
    main()
