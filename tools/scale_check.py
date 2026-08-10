#!/usr/bin/env python3
"""
다리 오도메트리 축척계수 측정

bag 앞뒤의 정지 구간을 자동으로 찾아, 그 사이의 직선 변위를 계산한다.
줄자(또는 타일) 실측값을 주면 축척계수까지 뽑는다.

사용 예
  python3 scale_check.py ~/data/bags/scale_0810_run1
  python3 scale_check.py ~/data/bags/scale_0810_run{1,2,3} --actual 21.84
  python3 scale_check.py ~/data/bags/scale_0810_run{1,2,3}_back --actual 21.84

주의: 실행 전 워크스페이스를 source 해야 한다. 로컬 bag 읽기이므로 srcoff 로 충분하다.
"""

import argparse
import math
import os
import statistics
import sys

import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("ROS2 환경이 source 되지 않았습니다. srcoff 실행 후 다시 시도하십시오.")


STOP_SPEED = 0.05   # m/s. 이 값 이하이면 정지로 간주
SPEED_WIN = 0.20    # s. 속도를 계산할 때 쓰는 시간 창


def storage_id(bag_dir):
    meta = os.path.join(bag_dir, "metadata.yaml")
    if os.path.isfile(meta):
        with open(meta) as f:
            m = yaml.safe_load(f)
        return m["rosbag2_bagfile_information"].get("storage_identifier", "sqlite3")
    return "sqlite3"


def read_odom(bag_dir, topic):
    so = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id(bag_dir))
    co = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(so, co)

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f"{bag_dir}\n  {topic} 가 없습니다. 담긴 토픽: {sorted(types)}")

    msg_cls = get_message(types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    rec = []
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        msg = deserialize_message(data, msg_cls)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        rec.append((t_ns * 1e-9, p.x, p.y, p.z, yaw))
    return rec


def speeds_of(rec):
    """SPEED_WIN 창으로 계산한 속도. 한 스텝 차분은 잡음이 커서 쓰지 않는다."""
    n = len(rec)
    sp = [0.0] * n
    j = 0
    for i in range(n):
        while j < i and rec[i][0] - rec[j][0] > SPEED_WIN:
            j += 1
        if j == i:
            continue
        dt = rec[i][0] - rec[j][0]
        if dt <= 0:
            continue
        sp[i] = math.hypot(rec[i][1] - rec[j][1], rec[i][2] - rec[j][2]) / dt
    return sp


def mean_pose(seg):
    return (statistics.fmean(r[1] for r in seg),
            statistics.fmean(r[2] for r in seg),
            statistics.fmean(r[3] for r in seg),
            statistics.fmean(r[4] for r in seg))


def analyse(bag_dir, topic):
    rec = read_odom(bag_dir, topic)
    if len(rec) < 20:
        raise SystemExit(f"{bag_dir}\n  샘플이 {len(rec)}개뿐입니다. 녹화가 제대로 안 된 것 같습니다.")

    sp = speeds_of(rec)
    moving = [s > STOP_SPEED for s in sp]
    if True not in moving:
        raise SystemExit(f"{bag_dir}\n  움직임이 감지되지 않았습니다.")

    i0 = moving.index(True)
    i1 = len(moving) - 1 - moving[::-1].index(True)

    head = rec[:i0] if i0 > 0 else rec[:1]
    tail = rec[i1 + 1:] if i1 + 1 < len(rec) else rec[-1:]

    x0, y0, z0, yaw0 = mean_pose(head)
    x1, y1, z1, yaw1 = mean_pose(tail)

    disp = math.hypot(x1 - x0, y1 - y0)

    path = 0.0
    for i in range(i0 + 1, i1 + 1):
        path += math.hypot(rec[i][1] - rec[i - 1][1], rec[i][2] - rec[i - 1][2])

    dyaw = math.degrees(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))

    return {
        "name": os.path.basename(bag_dir.rstrip("/")),
        "n": len(rec),
        "dur": rec[-1][0] - rec[0][0],
        "head_s": head[-1][0] - head[0][0],
        "tail_s": tail[-1][0] - tail[0][0],
        "move_s": rec[i1][0] - rec[i0][0],
        "disp": disp,
        "path": path,
        "dyaw": dyaw,
        "dz": z1 - z0,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--topic", default="/utlidar/robot_odom")
    ap.add_argument("--actual", type=float, default=None,
                    help="줄자·타일 실측 이동거리 [m]")
    args = ap.parse_args()

    results = []
    for b in args.bags:
        r = analyse(b, args.topic)
        results.append(r)

        print(f"\n{r['name']}")
        print(f"  샘플        {r['n']}개 · {r['dur']:.1f} s")
        print(f"  정지 구간   앞 {r['head_s']:.1f} s / 뒤 {r['tail_s']:.1f} s")
        print(f"  주행 시간   {r['move_s']:.1f} s   (평균 {r['path']/max(r['move_s'],1e-6):.2f} m/s)")
        print(f"  직선 변위   {r['disp']:.3f} m")
        print(f"  경로장      {r['path']:.3f} m   (직선 대비 {100*(r['path']/max(r['disp'],1e-6)-1):+.1f}%)")
        print(f"  yaw 변화    {r['dyaw']:+.1f} deg")
        print(f"  z 변화      {r['dz']:+.3f} m")

        if r["head_s"] < 1.0 or r["tail_s"] < 1.0:
            print("  ! 정지 구간이 1초 미만입니다. 시작·끝 자세 추정이 불안정할 수 있습니다.")
        if abs(r["dyaw"]) > 5.0:
            print("  ! 진행 중 방향이 5도 넘게 틀어졌습니다. 사선 주행 가능성.")

        if args.actual:
            k = args.actual / r["disp"]
            print(f"  실측 {args.actual:.2f} m  →  축척계수 {k:.4f}"
                  f"   (오도메트리가 {100*(1-r['disp']/args.actual):.2f}% 짧게 측정)")

    if len(results) > 1:
        d = [r["disp"] for r in results]
        print(f"\n{'='*46}")
        print(f"직선 변위  평균 {statistics.fmean(d):.3f} m"
              f"  편차 {statistics.pstdev(d):.3f} m"
              f"  (최대-최소 {max(d)-min(d):.3f} m)")
        if args.actual:
            ks = [args.actual / x for x in d]
            print(f"축척계수   평균 {statistics.fmean(ks):.4f}"
                  f"  편차 {statistics.pstdev(ks):.4f}")
            print(f"→ 지도 거리에 {statistics.fmean(ks):.4f} 를 곱하면 실제 거리")


if __name__ == "__main__":
    main()
