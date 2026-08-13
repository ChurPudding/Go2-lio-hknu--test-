#!/usr/bin/env python3
"""
drift_eval.py

rosbag2에 기록된 nav_msgs/Odometry로부터 폐루프 드리프트를 계산한다.
Point-LIO 출력과 leg odom을 같은 기준으로 비교하기 위한 도구.

전제: 시작점과 끝점이 물리적으로 같은 위치인 폐루프 주행 bag.

사용법
  단일:
    python3 drift_eval.py ~/data/exp/pl_0811_run1 /aft_mapped_to_init

  여러 토픽 동시 비교 (같은 bag 안에 둘 다 기록된 경우):
    python3 drift_eval.py ~/data/exp/pl_0811_run1 \
        /aft_mapped_to_init /utlidar/robot_odom

  반복 실행 결과 집계:
    python3 drift_eval.py --runs ~/data/exp/pl_0811_run*  --topic /aft_mapped_to_init

출력
  path_len      누적 이동 거리 [m]
  end_gap       시작-끝 위치 오차 [m]  (XY / Z 분리)
  drift         end_gap / path_len × 100 [%]
  yaw_gap       시작-끝 yaw 오차 [deg]
"""

import argparse
import glob
import math
import sys

import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
except ImportError:
    sys.exit("ROS2 환경을 먼저 source 하세요 (source /opt/ros/humble/setup.bash)")


def open_reader(bag_uri):
    last_err = None
    for sid in ("", "sqlite3", "mcap"):
        try:
            reader = SequentialReader()
            reader.open(
                StorageOptions(uri=bag_uri, storage_id=sid),
                ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr",
                ),
            )
            return reader
        except Exception as e:  # noqa: BLE001
            last_err = e
    raise RuntimeError(f"bag을 열 수 없습니다: {bag_uri}\n{last_err}")


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_poses(bag_uri, topics):
    """topic -> (Nx3 위치, N yaw, N 시각) """
    reader = open_reader(bag_uri)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    missing = [t for t in topics if t not in type_map]
    if missing:
        raise KeyError(
            f"토픽 없음: {', '.join(missing)}\nbag에 있는 토픽:\n  "
            + "\n  ".join(sorted(type_map))
        )

    cls = {t: get_message(type_map[t]) for t in topics}
    acc = {t: {"p": [], "yaw": [], "ts": []} for t in topics}

    while reader.has_next():
        tname, raw, stamp = reader.read_next()
        if tname not in acc:
            continue
        m = deserialize_message(raw, cls[tname])
        pos = m.pose.pose.position
        q = m.pose.pose.orientation
        acc[tname]["p"].append((pos.x, pos.y, pos.z))
        acc[tname]["yaw"].append(quat_to_yaw(q.x, q.y, q.z, q.w))
        acc[tname]["ts"].append(stamp * 1e-9)

    out = {}
    for t, d in acc.items():
        if not d["p"]:
            raise RuntimeError(f"'{t}' 메시지가 비어 있습니다.")
        out[t] = (
            np.asarray(d["p"], dtype=np.float64),
            np.asarray(d["yaw"], dtype=np.float64),
            np.asarray(d["ts"], dtype=np.float64),
        )
    return out


def evaluate(p, yaw, ts, tail=1):
    """tail: 끝점을 마지막 tail개 평균으로 잡아 노이즈 완화."""
    d = np.diff(p, axis=0)
    seg = np.linalg.norm(d, axis=1)
    path_len = float(seg.sum())

    start = p[0]
    end = p[-tail:].mean(axis=0) if tail > 1 else p[-1]

    gap_vec = end - start
    gap_xy = float(np.linalg.norm(gap_vec[:2]))
    gap_z = float(abs(gap_vec[2]))
    gap_3d = float(np.linalg.norm(gap_vec))

    yaw_gap = math.degrees(
        math.atan2(math.sin(yaw[-1] - yaw[0]), math.cos(yaw[-1] - yaw[0]))
    )

    drift = (gap_3d / path_len * 100.0) if path_len > 1e-6 else float("nan")

    return {
        "n": int(p.shape[0]),
        "dur": float(ts[-1] - ts[0]),
        "path_len": path_len,
        "gap_xy": gap_xy,
        "gap_z": gap_z,
        "gap_3d": gap_3d,
        "yaw_gap": yaw_gap,
        "drift": drift,
        "max_z": float(p[:, 2].max() - p[:, 2].min()),
    }


def print_row(label, r):
    print(f"  {label:<28}"
          f"{r['path_len']:>9.2f}"
          f"{r['gap_xy']:>9.3f}"
          f"{r['gap_z']:>8.3f}"
          f"{r['drift']:>9.2f}"
          f"{r['yaw_gap']:>10.2f}"
          f"{r['n']:>8}")


def header():
    print(f"  {'topic / run':<28}"
          f"{'path[m]':>9}{'gapXY[m]':>9}{'gapZ':>8}"
          f"{'drift%':>9}{'yaw[deg]':>10}{'msgs':>8}")
    print("  " + "-" * 73)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", nargs="?", help="rosbag2 디렉터리")
    ap.add_argument("topics", nargs="*", help="Odometry 토픽 (여러 개 가능)")
    ap.add_argument("--runs", nargs="+", help="반복 실행 bag들 (집계 모드)")
    ap.add_argument("--topic", default="/aft_mapped_to_init",
                    help="집계 모드에서 쓸 토픽")
    ap.add_argument("--tail", type=int, default=1,
                    help="끝점을 마지막 N개 평균으로 (기본 1)")
    args = ap.parse_args()

    if args.runs:
        runs = []
        for pat in args.runs:
            runs.extend(sorted(glob.glob(pat)) or [pat])
        print(f"\n[집계 모드] topic = {args.topic}\n")
        header()
        drifts = []
        for r in runs:
            try:
                data = load_poses(r, [args.topic])
                res = evaluate(*data[args.topic], tail=args.tail)
            except Exception as e:  # noqa: BLE001
                print(f"  {r.split('/')[-1]:<28} 실패: {e}")
                continue
            print_row(r.split("/")[-1], res)
            drifts.append(res["drift"])
        if len(drifts) >= 2:
            a = np.asarray(drifts)
            print("  " + "-" * 73)
            print(f"  drift  평균 {a.mean():.2f}%   표준편차 {a.std(ddof=1):.2f}%"
                  f"   최소 {a.min():.2f}%   최대 {a.max():.2f}%")
            if a.std(ddof=1) > 0.3 * a.mean():
                print("  [WARN] 회차 간 편차가 큽니다. 초기 회전 구간의 yaw 분기를")
                print("         의심해보세요 (기존에 확인된 재현성 이슈).")
        print()
        return

    if not args.bag:
        ap.print_help()
        sys.exit(1)

    topics = args.topics or ["/aft_mapped_to_init"]
    data = load_poses(args.bag, topics)

    print(f"\nbag: {args.bag}\n")
    header()
    for t in topics:
        print_row(t, evaluate(*data[t], tail=args.tail))
    print()

    if len(topics) >= 2:
        base = evaluate(*data[topics[0]], tail=args.tail)
        for t in topics[1:]:
            r = evaluate(*data[t], tail=args.tail)
            dl = r["path_len"] - base["path_len"]
            print(f"  경로길이 차이 ({t} - {topics[0]}) : {dl:+.2f} m "
                  f"({dl / base['path_len'] * 100:+.1f}%)")
        print()


if __name__ == "__main__":
    main()
