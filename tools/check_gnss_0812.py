#!/usr/bin/env python3
"""
check_gnss_0812.py — 0812 야외 bag의 /gnss 검증 + UERE 재산정

기존 gnss_bridge.py 는 UERE=3.0 으로 sigma를 산출한다.
그런데 0812 GPS 노이즈 분석 결론은 sigma = 5m (느린 드리프트 지배).
이 스크립트는 실제 bag의 hdop 분포를 읽어 두 값의 괴리를 수치로 보여준다.

노드/로봇 불필요. bag만 있으면 된다.

사용:
    srcoff
    python3 check_gnss_0812.py <bag_dir>
"""

import json
import math
import sys
from collections import Counter

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


TOPIC = "/gnss"

UERE_CURRENT = 3.0    # gnss_bridge.py 의 현재 값
SIGMA_TARGET = 5.0    # 0812 노이즈 분석 권고값


def read_gnss(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if TOPIC not in types:
        print(f"[FAIL] {TOPIC} 없음. bag에 녹화된 토픽:")
        for name in sorted(types):
            print(f"        {name}  ({types[name]})")
        sys.exit(1)

    print(f"[OK]   {TOPIC}  type={types[TOPIC]}")
    msg_cls = get_message(types[TOPIC])

    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))

    out = []
    while reader.has_next():
        _, raw, t_ns = reader.read_next()
        msg = deserialize_message(raw, msg_cls)
        try:
            d = json.loads(msg.data)
        except Exception:
            out.append((t_ns, None))
            continue
        out.append((t_ns, d))
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    rows = read_gnss(sys.argv[1])
    n = len(rows)
    if n == 0:
        print("[FAIL] /gnss 메시지 0개")
        sys.exit(1)

    bad = [t for t, d in rows if d is None]
    good = [(t, d) for t, d in rows if d is not None]

    hdops = [float(d["hdop"]) for _, d in good if d.get("hdop")]
    inuse = [int(d["satellite_inuse"]) for _, d in good if "satellite_inuse" in d]
    fixed = Counter(int(d.get("fixed", -1)) for _, d in good)

    invalid = sum(
        1 for _, d in good
        if int(d.get("fixed", 0)) == 0 or int(d.get("satellite_inuse", 0)) == 0
    )

    span_s = (rows[-1][0] - rows[0][0]) / 1e9
    rate = (n - 1) / span_s if span_s > 0 else 0.0

    print()
    print("=== /gnss 스트림 ===")
    print(f"  메시지        {n} 개  ({span_s:.1f} s, {rate:.2f} Hz)")
    print(f"  JSON 파싱실패 {len(bad)} 개")
    print(f"  무효(fix=0 또는 위성 0)  {invalid} / {len(good)}"
          f"  ({100.0*invalid/max(len(good),1):.1f}%)")
    print(f"  fixed 값 분포  {dict(fixed)}")

    if inuse:
        print(f"  사용 위성      {min(inuse)} ~ {max(inuse)}, 평균 {sum(inuse)/len(inuse):.1f}")

    if not hdops:
        print("[WARN] hdop 값이 하나도 없음 — 공분산 산출 불가")
        sys.exit(0)

    h_min, h_max = min(hdops), max(hdops)
    h_avg = sum(hdops) / len(hdops)
    h_sorted = sorted(hdops)
    h_p95 = h_sorted[int(0.95 * (len(h_sorted) - 1))]

    print(f"  HDOP           {h_min:.2f} ~ {h_max:.2f}, 평균 {h_avg:.2f}, p95 {h_p95:.2f}")

    print()
    print("=== 공분산 진단 ===")
    s_now = h_avg * UERE_CURRENT
    print(f"  현재 브리지 (UERE={UERE_CURRENT})   sigma = {s_now:.2f} m")
    print(f"  0812 노이즈 분석 권고             sigma = {SIGMA_TARGET:.2f} m")
    print(f"  괴리                              {SIGMA_TARGET/s_now:.2f} 배 과신")

    uere_new = SIGMA_TARGET / h_avg
    print()
    print(f"  -> 평균 HDOP {h_avg:.2f} 에서 sigma {SIGMA_TARGET:.1f} m 가 나오려면")
    print(f"     UERE = {uere_new:.2f}")
    print()
    print(f"  gnss_bridge.py 수정:  UERE 기본값 {UERE_CURRENT} -> {uere_new:.1f}")
    print(f"  (하한 2.0 m / 상한 25.0 m 클램프도 함께 두는 것을 권장)")

    # 실제로 브리지가 내보낼 값의 분포
    sig = sorted(min(max(h * uere_new, 2.0), 25.0) for h in hdops)
    print()
    print(f"  수정 후 sigma 분포   min {sig[0]:.2f}  "
          f"median {sig[len(sig)//2]:.2f}  max {sig[-1]:.2f} m")


if __name__ == "__main__":
    main()
