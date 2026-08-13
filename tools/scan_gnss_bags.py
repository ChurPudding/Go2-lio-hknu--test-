#!/usr/bin/env python3
"""
scan_gnss_bags.py — 여러 bag의 /gnss 품질을 한 줄씩 비교한다.

목적: 유선(랜투랜) 녹화본과 무선(공유기 부착) 녹화본의 GPS 품질 차이를
      새 실험 없이 기존 bag만으로 확인한다.

사용:
    srcoff
    python3 scan_gnss_bags.py ~/data/bags
    python3 scan_gnss_bags.py ~/data/bags/outdoor ~/data/bags/old
"""

import json
import os
import sys
import math

import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


TOPIC = "/gnss"
R_EARTH = 6378137.0


def find_bags(roots):
    """*.db3 를 재귀 탐색. 같은 bag의 분할 파일은 _0 만 대표로."""
    found = []
    for root in roots:
        root = os.path.expanduser(root)
        for dirpath, _, files in os.walk(root):
            for f in sorted(files):
                if not f.endswith(".db3"):
                    continue
                stem = f[:-4]
                if stem.endswith(tuple(f"_{i}" for i in range(1, 10))):
                    continue          # 분할 2번째 이후는 건너뜀
                found.append(os.path.join(dirpath, f))
    return sorted(found)


def probe(path):
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
    except Exception as ex:
        return {"err": str(ex)[:40]}

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if TOPIC not in types:
        return None

    cls = get_message(types[TOPIC])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))

    t, hdop, sat, lat, lon, invalid = [], [], [], [], [], 0
    while reader.has_next():
        _, raw, t_ns = reader.read_next()
        try:
            d = json.loads(deserialize_message(raw, cls).data)
        except Exception:
            continue
        fx = int(d.get("fixed", 0))
        iu = int(d.get("satellite_inuse", 0))
        if fx == 0 or iu == 0:
            invalid += 1
            continue
        t.append(t_ns / 1e9)
        hdop.append(float(d.get("hdop", 0.0)))
        sat.append(iu)
        lat.append(float(d.get("latitude", 0.0)))
        lon.append(float(d.get("longitude", 0.0)))

    if len(t) < 5:
        return {"n": len(t), "thin": True}

    t = np.array(t)
    hdop = np.array(hdop)
    sat = np.array(sat)
    lat = np.array(lat)
    lon = np.array(lon)

    # 이동 거리 (정지 bag인지 주행 bag인지 구분용)
    e = np.radians(lon - lon[0]) * R_EARTH * math.cos(math.radians(lat[0]))
    n = np.radians(lat - lat[0]) * R_EARTH
    span = float(np.hypot(e.max() - e.min(), n.max() - n.min()))

    dur = t[-1] - t[0]
    return {
        "n": len(t),
        "dur": dur,
        "hz": (len(t) - 1) / dur if dur > 0 else 0.0,
        "hdop_med": float(np.median(hdop)),
        "hdop_p95": float(np.percentile(hdop, 95)),
        "sat_med": float(np.median(sat)),
        "sat_min": int(sat.min()),
        "bad_pct": 100.0 * float((hdop > 2.0).mean()),
        "invalid": invalid,
        "span": span,
    }


def main():
    roots = sys.argv[1:] or ["~/data/bags"]
    bags = find_bags(roots)
    if not bags:
        print("db3 파일을 찾지 못했습니다. 경로를 확인해 주세요.")
        return

    print(f"{len(bags)} 개 db3 검사 중...\n")

    hdr = (f"{'bag':<44}{'n':>5}{'dur':>7}{'Hz':>6}"
           f"{'hdop med':>10}{'p95':>7}{'sat med':>9}{'min':>5}"
           f"{'>2.0%':>8}{'범위m':>8}")
    print(hdr)
    print("-" * len(hdr))

    rows = []
    for p in bags:
        r = probe(p)
        name = os.path.basename(p)
        if r is None:
            continue                      # /gnss 없음 — 조용히 건너뜀
        if "err" in r:
            print(f"{name:<44}  열기 실패: {r['err']}")
            continue
        if r.get("thin"):
            print(f"{name:<44}{r['n']:>5}   유효 fix 부족")
            continue
        print(f"{name:<44}{r['n']:>5}{r['dur']:>7.0f}{r['hz']:>6.2f}"
              f"{r['hdop_med']:>10.2f}{r['hdop_p95']:>7.2f}"
              f"{r['sat_med']:>9.1f}{r['sat_min']:>5d}"
              f"{r['bad_pct']:>8.1f}{r['span']:>8.0f}")
        rows.append((name, r))

    if not rows:
        print("\n/gnss 가 들어 있는 bag이 없습니다.")
        return

    print()
    print("읽는 법")
    print("  범위m 가 10 미만이면 정지 상태 bag — 간섭 비교에 가장 적합")
    print("  hdop med 와 sat med 를 유선/무선 bag끼리 비교할 것")
    print("  같은 장소·같은 날 bag끼리 비교해야 의미가 있음")


if __name__ == "__main__":
    main()
