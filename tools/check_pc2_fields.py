#!/usr/bin/env python3
"""
check_pc2_fields.py

PointCloud2 필드 레이아웃을 덤프하고, Point-LIO가 요구하는
"포인트별 타임스탬프"가 실제로 유효한 값인지 검사한다.

공식 L1 bag(/unilidar/cloud)과 Go2 bag(/utlidar/cloud)을 각각 돌린 뒤
--compare 로 두 결과를 대조하는 것이 목적.

사용법
  단일 검사:
    python3 check_pc2_fields.py ~/data/bags/exp/l1_official_ros2 /unilidar/cloud
    python3 check_pc2_fields.py ~/data/bags/indoor/loop_0810 /utlidar/cloud

  두 bag 비교:
    python3 check_pc2_fields.py \
        --compare ~/data/bags/exp/l1_official_ros2 /unilidar/cloud \
                  ~/data/bags/indoor/loop_0810      /utlidar/cloud

판정 기준
  [OK]   시간 필드가 존재하고, 한 스캔 안에서 값이 퍼져 있으며,
         그 폭이 스캔 주기(기본 0.1s)와 같은 자릿수다.
  [FAIL] 시간 필드가 없거나 / 전부 같은 값이거나 / 폭이 0에 가깝다.
         -> Point-LIO의 point-by-point 갱신이 무력화된 상태.
"""

import argparse
import sys

import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
except ImportError:
    sys.exit("ROS2 환경을 먼저 source 하세요 (source /opt/ros/humble/setup.bash)")


# sensor_msgs/PointField datatype -> numpy
DTYPE_MAP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}
DTYPE_NAME = {
    1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16",
    5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64",
}

# 드라이버별로 이름이 제각각이라 후보를 넓게 잡는다
TIME_CANDIDATES = (
    "time", "t", "timestamp", "time_stamp",
    "offset_time", "time_offset", "stamp",
)


def open_reader(bag_uri):
    """storage_id 자동 판별 (sqlite3 / mcap)."""
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


def read_messages(bag_uri, topic, n):
    reader = open_reader(bag_uri)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise KeyError(
            f"토픽 '{topic}' 없음. bag에 있는 토픽:\n  "
            + "\n  ".join(sorted(type_map))
        )

    msg_cls = get_message(type_map[topic])
    out = []
    while reader.has_next() and len(out) < n:
        tname, raw, _stamp = reader.read_next()
        if tname == topic:
            out.append(deserialize_message(raw, msg_cls))
    if not out:
        raise RuntimeError(f"'{topic}' 메시지를 하나도 읽지 못했습니다.")
    return out


def build_dtype(msg):
    """point_step을 itemsize로 하는 structured dtype 생성."""
    names, formats, offsets = [], [], []
    seen = {}
    for f in msg.fields:
        if f.datatype not in DTYPE_MAP:
            continue
        name = f.name
        # 중복 필드명 방어
        if name in seen:
            seen[name] += 1
            name = f"{name}_{seen[name]}"
        else:
            seen[name] = 0
        base = DTYPE_MAP[f.datatype]
        names.append(name)
        formats.append(base if f.count == 1 else (base, f.count))
        offsets.append(f.offset)
    return np.dtype(
        {"names": names, "formats": formats,
         "offsets": offsets, "itemsize": msg.point_step}
    )


def to_array(msg):
    dt = build_dtype(msg)
    n = msg.width * msg.height
    return np.frombuffer(msg.data, dtype=dt, count=n)


def describe(bag_uri, topic, n_msgs, scan_period):
    msgs = read_messages(bag_uri, topic, n_msgs)
    m0 = msgs[0]

    print("=" * 68)
    print(f"bag   : {bag_uri}")
    print(f"topic : {topic}   (읽은 메시지 {len(msgs)}개)")
    print("=" * 68)

    print(f"frame_id     : {m0.header.frame_id}")
    print(f"height×width : {m0.height} × {m0.width}")
    print(f"point_step   : {m0.point_step} bytes")
    print(f"is_dense     : {m0.is_dense}")
    print(f"is_bigendian : {m0.is_bigendian}"
          + ("   <- 빅엔디언, 아래 수치 신뢰 불가" if m0.is_bigendian else ""))

    print("\n[필드 레이아웃]")
    print(f"  {'name':<14}{'offset':>7}{'datatype':>10}{'count':>7}")
    for f in m0.fields:
        print(f"  {f.name:<14}{f.offset:>7}"
              f"{DTYPE_NAME.get(f.datatype, f.datatype):>10}{f.count:>7}")

    fnames = [f.name for f in m0.fields]

    # ---- 시간 필드 검사 -------------------------------------------------
    tfield = next((c for c in TIME_CANDIDATES if c in fnames), None)
    print("\n[포인트별 타임스탬프]")
    if tfield is None:
        print("  [FAIL] 시간 필드 없음 "
              f"(찾은 후보: {', '.join(TIME_CANDIDATES)})")
        print("         Point-LIO의 point-by-point 갱신이 동작하지 않습니다.")
    else:
        spans, uniq_cnt, npts = [], [], []
        for m in msgs:
            a = to_array(m)
            v = np.asarray(a[tfield], dtype=np.float64)
            if v.size == 0:
                continue
            spans.append(float(v.max() - v.min()))
            uniq_cnt.append(int(np.unique(v).size))
            npts.append(int(v.size))

        span = float(np.mean(spans)) if spans else 0.0
        # 고유값 개수와 포인트 수는 반드시 같은 메시지 집합의 평균이어야 한다.
        # (메시지마다 포인트 수가 달라 분모/분자를 섞으면 비율이 100%를 넘는다)
        uniq_n = float(np.mean(uniq_cnt)) if uniq_cnt else 0.0
        n_mean = float(np.mean(npts)) if npts else 0.0
        uniq_ratio = (uniq_n / n_mean) if n_mean > 0 else 0.0

        a0 = to_array(m0)
        v0 = np.asarray(a0[tfield], dtype=np.float64)
        print(f"  필드명       : {tfield}")
        print(f"  첫 5개 값    : {np.array2string(v0[:5], precision=9)}")
        print(f"  min / max    : {v0.min():.9g} / {v0.max():.9g}")
        print(f"  스캔 내 폭   : {span:.9g}  (평균)")
        print(f"  포인트 수    : {min(npts)}~{max(npts)} "
              f"(평균 {n_mean:.0f}, 메시지 {len(npts)}개)")
        print(f"  고유값       : 평균 {uniq_n:.0f} / {n_mean:.0f} "
              f"({uniq_ratio * 100:.1f}%)")

        if uniq_n <= 1:
            print("  [FAIL] 모든 포인트의 시간이 동일합니다. 디스큐 불가.")
        elif span <= 1e-9:
            print("  [FAIL] 시간 폭이 사실상 0입니다.")
        else:
            # 폭이 스캔주기와 같은 자릿수인지 (초 단위 / 나노초 단위 모두 허용)
            ratio_s = span / scan_period
            ratio_ns = (span * 1e-9) / scan_period
            if 0.2 <= ratio_s <= 5.0:
                print(f"  [OK] 초 단위로 보입니다 (주기 대비 {ratio_s:.2f}배)")
            elif 0.2 <= ratio_ns <= 5.0:
                print(f"  [OK] 나노초 단위로 보입니다 (주기 대비 {ratio_ns:.2f}배)")
                print("       -> config의 시간 단위 설정을 확인하세요.")
            else:
                print(f"  [WARN] 폭이 스캔 주기({scan_period}s)와 맞지 않습니다. "
                      f"단위 또는 스케일 확인 필요.")

            if 0 < uniq_ratio < 0.1:
                print(f"  [주의] 고유값 비율이 {uniq_ratio * 100:.1f}%로 낮습니다. "
                      "시간 해상도가 거칠어 디스큐 효과가 제한될 수 있습니다.")

    # ---- 중복 포인트 검사 (Go2 cloud_deskewed 기존 이슈) ------------------
    print("\n[중복 포인트]")
    if all(k in fnames for k in ("x", "y", "z")):
        rates = []
        for m in msgs:
            a = to_array(m)
            xyz = np.stack(
                [np.asarray(a["x"], dtype=np.float64),
                 np.asarray(a["y"], dtype=np.float64),
                 np.asarray(a["z"], dtype=np.float64)],
                axis=1,
            )
            total = xyz.shape[0]
            if total == 0:
                continue
            uniq_n = np.unique(xyz, axis=0).shape[0]
            rates.append((total - uniq_n) / total)
        if rates:
            r = float(np.mean(rates))
            tag = "[WARN]" if r > 0.05 else "[OK]"
            print(f"  {tag} 중복률 {r*100:.1f}% (메시지 {len(rates)}개 평균)")
    else:
        print("  x/y/z 필드가 없어 건너뜁니다.")

    print()
    return {
        "fields": [(f.name, f.offset, f.datatype, f.count) for f in m0.fields],
        "point_step": m0.point_step,
        "time_field": tfield,
        "frame_id": m0.header.frame_id,
    }


def compare(a, b):
    print("=" * 68)
    print("비교 결과")
    print("=" * 68)

    fa = {n: (o, d, c) for n, o, d, c in a["fields"]}
    fb = {n: (o, d, c) for n, o, d, c in b["fields"]}

    only_a = sorted(set(fa) - set(fb))
    only_b = sorted(set(fb) - set(fa))
    both = sorted(set(fa) & set(fb))

    if only_a:
        print(f"  A에만 있는 필드 : {', '.join(only_a)}")
    if only_b:
        print(f"  B에만 있는 필드 : {', '.join(only_b)}")

    mismatch = [n for n in both if fa[n] != fb[n]]
    if mismatch:
        print("  레이아웃 불일치 :")
        for n in mismatch:
            oa, da, ca = fa[n]
            ob, db, cb = fb[n]
            print(f"    {n:<12} A(off={oa}, {DTYPE_NAME.get(da,da)}, cnt={ca})"
                  f"  vs  B(off={ob}, {DTYPE_NAME.get(db,db)}, cnt={cb})")

    if a["point_step"] != b["point_step"]:
        print(f"  point_step 불일치 : {a['point_step']} vs {b['point_step']}")

    print()
    if not (only_a or only_b or mismatch) and a["point_step"] == b["point_step"]:
        print("  => 레이아웃 동일. lidar_type:5 전처리를 그대로 써도 됩니다.")
    else:
        print("  => 레이아웃 다름. preprocess.cpp의 UNILIDAR 분기가 Go2 데이터를")
        print("     잘못 파싱하고 있을 가능성이 큽니다. 드리프트의 유력 원인입니다.")

    if a["time_field"] != b["time_field"]:
        print(f"  => 시간 필드명 다름: '{a['time_field']}' vs '{b['time_field']}'")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag", nargs="?", help="rosbag2 디렉터리")
    p.add_argument("topic", nargs="?", help="PointCloud2 토픽")
    p.add_argument("--compare", nargs=4,
                   metavar=("BAG_A", "TOPIC_A", "BAG_B", "TOPIC_B"),
                   help="두 bag의 레이아웃을 대조")
    p.add_argument("-n", "--num", type=int, default=20,
                   help="검사할 메시지 수 (기본 20)")
    p.add_argument("--period", type=float, default=0.1,
                   help="스캔 주기 [s] (기본 0.1 = 10Hz)")
    args = p.parse_args()

    if args.compare:
        ba, ta, bb, tb = args.compare
        ra = describe(ba, ta, args.num, args.period)
        rb = describe(bb, tb, args.num, args.period)
        compare(ra, rb)
    elif args.bag and args.topic:
        describe(args.bag, args.topic, args.num, args.period)
    else:
        p.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
