#!/usr/bin/env python3
"""
gnss_bridge.py — Go2 /gnss (JSON string) -> sensor_msgs/NavSatFix

bag 재생만으로 검증 가능한 순수 변환 노드.
공분산은 hdop 기반으로 계산하며, 이상치 억제(Huber 등)는 여기서 하지 않고
GTSAM 팩터 그래프 쪽에 맡긴다.

실행:
    # 터미널 1
    srcoff
    ros2 bag play <bag> -r 0.5 --clock

    # 터미널 2
    srcoff
    python3 gnss_bridge.py --ros-args -p use_sim_time:=true

    # 터미널 3
    ros2 topic echo /fix
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus


# ---------------------------------------------------------------------------
# 필드 이름 매핑. `ros2 topic echo /gnss --once` 결과를 보고 여기만 수정하면 된다.
# 왼쪽이 우리가 쓰는 이름, 오른쪽이 JSON에서 찾아볼 후보 키들(앞에서부터 탐색).
# ---------------------------------------------------------------------------
KEY_ALIASES = {
    "lat":  ["lat", "latitude", "Lat", "Latitude"],
    "lon":  ["lon", "lng", "longitude", "Lon", "Longitude"],
    "alt":  ["alt", "altitude", "height", "Alt", "Altitude"],
    "hdop": ["hdop", "HDOP", "hDop", "dop"],
    "fix":  ["fixed", "fix", "fix_type", "quality", "status"],
    "sats": ["satellites", "sat_num", "num_sats", "nsat", "sats"],
    "time": ["timestamp", "time", "utc", "stamp"],
}

# hdop -> 수평 표준편차(m). sigma = hdop * UERE
# 실측 hdop 1.2 기준으로 약 4.8m가 되도록 UERE=4.0 사용 (권고 σ≈5m와 일치)
UERE = 4.0
SIGMA_MIN = 2.0      # 과신 방지 하한
SIGMA_MAX = 25.0     # 발산 방지 상한
VERT_FACTOR = 1.8    # 수직 오차는 수평의 약 1.8배
HDOP_FALLBACK = 2.0  # hdop 필드가 없을 때 보수적으로 가정


def pick(d, name):
    """KEY_ALIASES 기준으로 값을 꺼낸다. 없으면 None."""
    for key in KEY_ALIASES[name]:
        if key in d and d[key] is not None:
            return d[key]
    return None


def to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


class GnssBridge(Node):

    def __init__(self):
        super().__init__("gnss_bridge")

        self.declare_parameter("input_topic", "/gnss")
        self.declare_parameter("output_topic", "/fix")
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("use_json_time", False)

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.use_json_time = self.get_parameter("use_json_time").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(NavSatFix, self.out_topic, qos)
        self.sub = self.create_subscription(String, self.in_topic, self.on_gnss, qos)

        self.n_in = 0
        self.n_out = 0
        self.n_bad = 0
        self.warned_keys = set()
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f"gnss_bridge: {self.in_topic} -> {self.out_topic} (frame={self.frame_id})"
        )

    # ---------------------------------------------------------------- 콜백
    def on_gnss(self, msg: String):
        self.n_in += 1

        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.n_bad += 1
            self.warn_once("json", "JSON 파싱 실패 — 메시지 타입이 String이 맞는지 확인")
            return

        if not isinstance(data, dict):
            self.n_bad += 1
            self.warn_once("dict", f"최상위가 dict가 아님: {type(data).__name__}")
            return

        lat = to_float(pick(data, "lat"))
        lon = to_float(pick(data, "lon"))
        if lat is None or lon is None:
            self.n_bad += 1
            self.warn_once(
                "latlon",
                f"위경도 키를 못 찾음. 실제 키 목록: {sorted(data.keys())} "
                f"-> KEY_ALIASES 수정 필요",
            )
            return

        alt = to_float(pick(data, "alt")) or 0.0
        hdop = to_float(pick(data, "hdop"))
        if hdop is None or hdop <= 0.0:
            hdop = HDOP_FALLBACK
            self.warn_once("hdop", f"hdop 없음 — {HDOP_FALLBACK} 로 가정")

        fix_raw = pick(data, "fix")
        sats = to_float(pick(data, "sats"))

        out = NavSatFix()

        # --- 타임스탬프 -----------------------------------------------------
        # 기본은 현재 시각. bag 재생 시 --clock + use_sim_time:=true 를 쓰면
        # 이 시각이 bag 원본 시각과 일치한다. JSON에 유닉스 시각이 있으면
        # use_json_time:=true 로 그쪽을 우선 사용.
        stamped = False
        if self.use_json_time:
            t = to_float(pick(data, "time"))
            if t is not None and t > 1.0e8:
                if t > 1.0e12:      # ms 단위로 보임
                    t /= 1000.0
                out.header.stamp.sec = int(t)
                out.header.stamp.nanosec = int((t - int(t)) * 1e9)
                stamped = True
        if not stamped:
            out.header.stamp = self.get_clock().now().to_msg()

        out.header.frame_id = self.frame_id

        # --- 상태 -----------------------------------------------------------
        out.status.service = NavSatStatus.SERVICE_GPS
        out.status.status = self.map_status(fix_raw, sats)

        out.latitude = lat
        out.longitude = lon
        out.altitude = alt

        # --- 공분산 ---------------------------------------------------------
        sigma_h = min(max(hdop * UERE, SIGMA_MIN), SIGMA_MAX)
        sigma_v = sigma_h * VERT_FACTOR
        out.position_covariance = [
            sigma_h ** 2, 0.0, 0.0,
            0.0, sigma_h ** 2, 0.0,
            0.0, 0.0, sigma_v ** 2,
        ]
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(out)
        self.n_out += 1

    # ---------------------------------------------------------------- 보조
    def map_status(self, fix_raw, sats):
        """fix 필드를 NavSatStatus로. 값 체계는 로그로 확인 후 조정."""
        if sats is not None and sats < 4:
            return NavSatStatus.STATUS_NO_FIX

        f = to_float(fix_raw)
        if f is None:
            # 문자열로 오는 경우
            s = str(fix_raw).lower()
            if s in ("rtk", "rtk_fixed", "4", "5"):
                return NavSatStatus.STATUS_GBAS_FIX
            if s in ("dgps", "2"):
                return NavSatStatus.STATUS_SBAS_FIX
            if s in ("gps", "1", "true"):
                return NavSatStatus.STATUS_FIX
            return NavSatStatus.STATUS_NO_FIX

        if f <= 0:
            return NavSatStatus.STATUS_NO_FIX
        if f >= 4:
            return NavSatStatus.STATUS_GBAS_FIX
        if f == 2:
            return NavSatStatus.STATUS_SBAS_FIX
        return NavSatStatus.STATUS_FIX

    def warn_once(self, tag, text):
        if tag not in self.warned_keys:
            self.warned_keys.add(tag)
            self.get_logger().warn(text)

    def report(self):
        if self.n_in == 0:
            self.get_logger().warn(f"{self.in_topic} 메시지 수신 없음")
            return
        self.get_logger().info(
            f"in={self.n_in} out={self.n_out} bad={self.n_bad}"
        )


def main():
    rclpy.init()
    node = GnssBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
