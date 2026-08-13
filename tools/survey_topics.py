#!/usr/bin/env python3
"""
survey_topics.py — 살아 있는 토픽을 전부 훑어 주기와 샘플 내용을 기록한다.

ros2 topic echo --once 를 하나씩 치는 것보다 안전하다.
  - 발행자 0인 토픽은 애초에 건드리지 않는다 (무한 대기 방지)
  - 점군/이미지는 헤더와 크기만 기록한다 (터미널 마비 방지)
  - 결과를 파일로 남겨 나중에 비교할 수 있다

사용:
    source ~/unitree_ros2/setup_go2.sh      # 로봇 연결 상태에서
    python3 survey_topics.py                # 전체
    python3 survey_topics.py --filter utlidar   # 이름에 utlidar 포함만
    python3 survey_topics.py --dur 5.0      # 토픽당 관찰 시간 (기본 3초)
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message


# 내용을 요약만 할 타입 (그대로 찍으면 터미널이 죽는다)
HEAVY = (
    "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "nav_msgs/msg/OccupancyGrid",
    "visualization_msgs/msg/MarkerArray",
)

# 조사에서 뺄 토픽 (잡음)
SKIP_PREFIX = ("/parameter_events", "/rosout")


def summarize(msg, type_name, max_chars=700):
    """무거운 타입은 핵심만, 나머지는 YAML을 잘라서."""
    if type_name == "sensor_msgs/msg/PointCloud2":
        return (f"frame={msg.header.frame_id} "
                f"{msg.width}x{msg.height} pts, "
                f"point_step={msg.point_step}, "
                f"fields={[f.name for f in msg.fields]}")
    if type_name in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
        if hasattr(msg, "width"):
            return f"frame={msg.header.frame_id} {msg.width}x{msg.height} {msg.encoding}"
        return f"frame={msg.header.frame_id} format={msg.format} {len(msg.data)} bytes"
    if type_name == "nav_msgs/msg/OccupancyGrid":
        i = msg.info
        return (f"frame={msg.header.frame_id} {i.width}x{i.height} "
                f"res={i.resolution} origin=({i.origin.position.x:.2f}, "
                f"{i.origin.position.y:.2f})")
    if type_name == "visualization_msgs/msg/MarkerArray":
        return f"{len(msg.markers)} markers"

    text = message_to_yaml(msg).strip()
    if len(text) > max_chars:
        text = text[:max_chars] + "\n      ... (생략)"
    return "\n      " + text.replace("\n", "\n      ")


class Surveyor(Node):
    def __init__(self):
        super().__init__("topic_surveyor")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dur", type=float, default=3.0, help="토픽당 관찰 시간 [s]")
    ap.add_argument("--filter", default="", help="토픽 이름 부분 일치")
    ap.add_argument("--out", default="topic_survey.txt")
    args = ap.parse_args()

    rclpy.init()
    node = Surveyor()
    time.sleep(1.5)                       # discovery 대기

    names = node.get_topic_names_and_types()
    lines = []

    def emit(s=""):
        print(s)
        lines.append(s)

    targets = []
    for name, types in sorted(names):
        if name.startswith(SKIP_PREFIX):
            continue
        if args.filter and args.filter not in name:
            continue
        n_pub = node.count_publishers(name)
        targets.append((name, types[0], n_pub))

    live = [t for t in targets if t[2] > 0]
    dead = [t for t in targets if t[2] == 0]

    emit(f"전체 {len(targets)} 개  |  발행자 있음 {len(live)}  "
         f"|  발행자 없음 {len(dead)}")
    emit(f"토픽당 {args.dur}s 관찰\n")

    if dead:
        emit("=" * 70)
        emit("발행자 없음 — 데이터가 흐르지 않음")
        emit("=" * 70)
        for name, tp, _ in dead:
            emit(f"  {name}   ({tp})")
        emit("")

    emit("=" * 70)
    emit("발행 중인 토픽")
    emit("=" * 70)

    for name, tp, n_pub in live:
        try:
            cls = get_message(tp)
        except Exception as ex:
            emit(f"\n[{name}]  타입 로드 실패: {ex}")
            continue

        got = {"n": 0, "last": None}

        def cb(msg, store=got):
            store["n"] += 1
            store["last"] = msg

        # 발행자 QoS에 맞춰 구독 (BEST_EFFORT 발행자도 잡히도록)
        subs = []
        for rel in (ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE):
            qos = QoSProfile(
                reliability=rel,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                durability=DurabilityPolicy.VOLATILE,
            )
            try:
                subs.append(node.create_subscription(cls, name, cb, qos))
            except Exception:
                pass

        t0 = time.time()
        while time.time() - t0 < args.dur:
            rclpy.spin_once(node, timeout_sec=0.05)
        span = time.time() - t0

        for s in subs:
            node.destroy_subscription(s)

        if got["n"] == 0:
            emit(f"\n[{name}]  pub={n_pub}  {tp}")
            emit(f"  수신 0 — 이벤트성 토픽이거나 QoS 불일치")
            continue

        hz = got["n"] / span
        emit(f"\n[{name}]  pub={n_pub}  {tp}")
        emit(f"  {got['n']} msgs / {span:.1f}s  ≈ {hz:.1f} Hz")
        emit(f"  sample: {summarize(got['last'], tp)}")

    with open(args.out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\n저장: {args.out}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
