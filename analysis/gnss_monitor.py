#!/usr/bin/env python3
"""
gnss_monitor.py — Go2 내장 GPS(/gnss, std_msgs/String JSON) 실시간 연결·상태 로거

야외 루프 테스트용. 로봇이 실제로 GPS를 보내고 있는지, fix가 잡혔는지,
위성 수·hdop이 어떤지 콘솔에 실시간으로 찍고 CSV로도 남긴다.
메시지가 끊기면 경고하고, fix 획득/상실 순간을 로그로 남긴다.

실행:
    python3 gnss_monitor.py
    python3 gnss_monitor.py --topic /gnss --csv my_log.csv --stale 3
"""
import argparse
import csv
import json
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _get(d, keys, default=None):
    """여러 후보 키 중 먼저 있는 값을 반환 (JSON 필드명 방어적 처리)."""
    for k in keys:
        if k in d and d[k] is not None:
            return d[k]
    return default


class GnssMonitor(Node):
    def __init__(self, topic, csv_path, stale_sec):
        super().__init__('gnss_monitor')
        self.stale_sec = stale_sec

        # 상태 추적용
        self.count = 0
        self.parse_err = 0
        self.last_msg_wall = None      # 마지막 메시지 수신 시각(초)
        self.connected = False         # 메시지가 흐르고 있는가
        self.last_fixed = None         # 직전 fix 상태 (전이 감지용)
        self.hdop_min = None
        self.hdop_max = None
        self.sat_max = 0
        self.got_fix = False

        # CSV 준비 (수신 즉시 한 줄씩 기록)
        self.csv_path = csv_path
        self.csv_file = open(csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            ['wall_time', 'fixed', 'sat_total', 'sat_inuse',
             'hdop', 'latitude', 'longitude', 'altitude'])

        self.sub = self.create_subscription(String, topic, self.cb, 10)
        # 1초마다 연결 상태(끊김) 감시
        self.timer = self.create_timer(1.0, self.watchdog)

        self.get_logger().info(f"'{topic}' 구독 시작. CSV -> {csv_path}")
        self.get_logger().info(f"메시지가 {stale_sec:.0f}초 이상 안 오면 '연결 끊김'으로 봅니다.")

    def cb(self, msg):
        now = time.time()
        self.last_msg_wall = now
        if not self.connected:
            self.connected = True
            self.get_logger().info("● GPS 메시지 수신 시작 (연결 OK)")

        # /gnss 는 JSON 문자열이라 파싱이 필요
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.parse_err += 1
            if self.parse_err <= 3:
                self.get_logger().warn(f"JSON 파싱 실패 (원문 일부): {msg.data[:80]!r}")
            return

        fixed = int(_get(d, ['fixed', 'fix'], 0) or 0)
        sat_total = int(_get(d, ['satellite_total', 'sat_total', 'total'], 0) or 0)
        sat_inuse = int(_get(d, ['satellite_inuse', 'sat_inuse', 'inuse'], 0) or 0)
        hdop = float(_get(d, ['hdop'], 0.0) or 0.0)
        lat = float(_get(d, ['latitude', 'lat'], 0.0) or 0.0)
        lon = float(_get(d, ['longitude', 'lon'], 0.0) or 0.0)
        alt = float(_get(d, ['altitude', 'alt', 'height'], 0.0) or 0.0)

        self.count += 1
        self.sat_max = max(self.sat_max, sat_inuse)
        if hdop > 0:
            self.hdop_min = hdop if self.hdop_min is None else min(self.hdop_min, hdop)
            self.hdop_max = hdop if self.hdop_max is None else max(self.hdop_max, hdop)

        # fix 상태가 바뀌는 순간만 따로 로그
        if self.last_fixed is not None and fixed != self.last_fixed:
            if fixed >= 1:
                self.got_fix = True
                self.get_logger().info(
                    f"✔ FIX 획득 (사용 위성 {sat_inuse}개, hdop {hdop:.1f})")
            else:
                self.get_logger().warn("✖ FIX 상실 (fixed=0)")
        elif self.last_fixed is None and fixed >= 1:
            self.got_fix = True
        self.last_fixed = fixed

        wall = datetime.now().strftime('%H:%M:%S')
        state = 'FIX  ' if fixed >= 1 else 'NO-FIX'
        self.get_logger().info(
            f"[{wall}] {state} | 위성 {sat_inuse}/{sat_total} | hdop {hdop:.1f} "
            f"| {lat:.6f}, {lon:.6f}")

        self.writer.writerow(
            [datetime.now().isoformat(timespec='seconds'), fixed, sat_total,
             sat_inuse, f"{hdop:.2f}", f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}"])
        self.csv_file.flush()

    def watchdog(self):
        """메시지가 일정 시간 이상 안 오면 끊김으로 경고."""
        if self.last_msg_wall is None:
            self.get_logger().warn("아직 GPS 메시지가 한 번도 안 왔습니다. 토픽/연결 확인 필요.")
            return
        gap = time.time() - self.last_msg_wall
        if gap > self.stale_sec and self.connected:
            self.connected = False
            self.get_logger().error(f"⚠ GPS 끊김: {gap:.1f}초째 메시지 없음")

    def summary(self):
        self.get_logger().info("──── 요약 ────")
        self.get_logger().info(f"총 메시지 {self.count}개, 파싱 실패 {self.parse_err}개")
        if self.hdop_min is not None:
            self.get_logger().info(
                f"hdop 범위 {self.hdop_min:.1f}~{self.hdop_max:.1f}, "
                f"최대 사용 위성 {self.sat_max}개")
        self.get_logger().info(
            "FIX 확인됨" if self.got_fix else "이번 세션에서 FIX(fixed=1)는 없었습니다.")
        self.get_logger().info(f"CSV 저장: {self.csv_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/gnss')
    parser.add_argument('--csv', default=None)
    parser.add_argument('--stale', type=float, default=3.0,
                        help="이 초 이상 메시지가 없으면 끊김으로 판단")
    args = parser.parse_args()

    csv_path = args.csv or f"gnss_log_{datetime.now():%Y%m%d_%H%M}.csv"

    rclpy.init()
    node = GnssMonitor(args.topic, csv_path, args.stale)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
