#!/usr/bin/env python3
"""
gnss_bridge.py  --  Go2 /gnss (JSON 문자열) -> sensor_msgs/NavSatFix 브리지

배경
----
Go2 의 /gnss 는 std_msgs/String 에 JSON 을 담아 발행한다.

    {"fixed":1,"hdop":0.800000,"longitude":127.264220,"latitude":37.011434,
     "satellite_total":12,"satellite_inuse":9,"timestamp":1785397828}

robot_localization 의 navsat_transform_node 는 sensor_msgs/NavSatFix 를 요구하므로
그대로는 쓸 수 없다. 이 노드가 변환해 /gps/fix 로 발행한다.

실측 근거 (rosbag2_2026_07_30-16_50_28, 실외 944초, 477개)
--------------------------------------------------------
  무효(fixed=0 또는 위성 0)  1 / 477 = 0.2%
  HDOP        0.70 ~ 1.20, 평균 0.78
  사용 위성    5 ~ 9,       평균 7.8
  발행 주기    1.98 초 (약 0.5 Hz)
  최대 속도    2.67 m/s  (GPS 튐 없음. Go2 보행 속도 범위)

공분산
------
NavSatFix 의 position_covariance 는 분산[m^2] 이다. 표준 GPS 에서

    sigma_h = HDOP * UERE

UERE(User Equivalent Range Error)는 수신기·환경에 따라 3~5 m 이다. 기본 3.0 을
쓰면 HDOP 0.78 에서 sigma_h ~ 2.3 m 가 되어 실측 감각과 맞는다. 값을 키우면
EKF 가 GPS 를 덜 믿는다. 튜닝 손잡이로 쓸 것.

수직은 수평보다 나쁘므로 vert_factor 배(기본 2.0)로 둔다. 단 이 JSON 에는
고도가 없으므로 altitude 는 NaN 이고, 수직 공분산은 형식상 채우는 값이다.

타임스탬프 주의
--------------
JSON 의 timestamp 는 1초 해상도인데 메시지는 약 2초에 하나씩 온다(실측). 즉
측위 시각과 발행 시각이 어긋난다. 기본값은 수신 시각(now)이며, 필요하면
stamp_source:=json 으로 바꿀 수 있다. EKF 시간 동기가 이상하면 여기를 의심할 것.

사용
----
    python3 gnss_bridge.py
    python3 gnss_bridge.py --ros-args -p uere:=4.0 -p stamp_source:=json

    ros2 topic echo /gps/fix --once
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

COV_TYPE_APPROXIMATED = 2      # NavSatFix.COVARIANCE_TYPE_APPROXIMATED


class GnssBridge(Node):
    def __init__(self):
        super().__init__('gnss_bridge')

        self.declare_parameter('in_topic', '/gnss')
        self.declare_parameter('out_topic', '/gps/fix')
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('uere', 3.0)          # HDOP 1 당 수평 오차 [m]
        self.declare_parameter('vert_factor', 2.0)   # 수직 = 수평 * 이 값
        self.declare_parameter('stamp_source', 'now')  # 'now' | 'json'
        self.declare_parameter('min_satellites', 4)
        self.declare_parameter('max_hdop', 5.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.uere = float(self.get_parameter('uere').value)
        self.vert_factor = float(self.get_parameter('vert_factor').value)
        self.stamp_source = str(self.get_parameter('stamp_source').value)
        self.min_sat = int(self.get_parameter('min_satellites').value)
        self.max_hdop = float(self.get_parameter('max_hdop').value)

        self.n_in = 0
        self.n_fix = 0
        self.n_nofix = 0
        self.n_bad = 0        # JSON 파싱 실패
        self.last_hdop = None
        self.last_sat = None

        self.pub = self.create_publisher(
            NavSatFix, self.get_parameter('out_topic').value, qos_profile_sensor_data)
        self.create_subscription(
            String, self.get_parameter('in_topic').value,
            self.on_gnss, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)

        self.get_logger().info(
            'gnss_bridge %s -> %s  (uere=%.1f, stamp=%s)'
            % (self.get_parameter('in_topic').value,
               self.get_parameter('out_topic').value,
               self.uere, self.stamp_source))

    # ------------------------------------------------------------------
    def on_gnss(self, msg):
        self.n_in += 1
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            self.n_bad += 1
            if self.n_bad <= 3:
                self.get_logger().warn('JSON 파싱 실패: %r' % msg.data[:80])
            return

        try:
            lat = float(d['latitude'])
            lon = float(d['longitude'])
            hdop = float(d.get('hdop', 0.0))
            fixed = int(d.get('fixed', 0))
            sat_use = int(d.get('satellite_inuse', 0))
        except (KeyError, ValueError, TypeError):
            self.n_bad += 1
            return

        self.last_hdop, self.last_sat = hdop, sat_use

        out = NavSatFix()

        # --- 타임스탬프 -------------------------------------------------
        if self.stamp_source == 'json' and 'timestamp' in d:
            ts = float(d['timestamp'])
            out.header.stamp.sec = int(ts)
            out.header.stamp.nanosec = int((ts - int(ts)) * 1e9)
        else:
            out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        # --- 유효성 판정 ------------------------------------------------
        # 실측에서 무효는 fixed=0 또는 위성 0 으로 나타났다.
        # hdop=0.0 도 유효값이 아니라 측위 실패 시 나오는 값이다.
        valid = (fixed == 1
                 and sat_use >= self.min_sat
                 and 0.0 < hdop <= self.max_hdop)

        out.status.service = NavSatStatus.SERVICE_GPS
        if valid:
            out.status.status = NavSatStatus.STATUS_FIX
            self.n_fix += 1
        else:
            # navsat_transform_node 는 STATUS_NO_FIX 를 무시한다.
            out.status.status = NavSatStatus.STATUS_NO_FIX
            self.n_nofix += 1

        out.latitude = lat
        out.longitude = lon
        out.altitude = float('nan')      # JSON 에 고도 없음

        # --- 공분산 -----------------------------------------------------
        if valid:
            sigma_h = hdop * self.uere
        else:
            sigma_h = 1.0e3              # 사실상 무한대
        sigma_v = sigma_h * self.vert_factor

        out.position_covariance = [
            sigma_h ** 2, 0.0, 0.0,
            0.0, sigma_h ** 2, 0.0,
            0.0, 0.0, sigma_v ** 2,
        ]
        out.position_covariance_type = COV_TYPE_APPROXIMATED

        self.pub.publish(out)

    # ------------------------------------------------------------------
    def report(self):
        if self.n_in == 0:
            self.get_logger().warn(
                '/gnss 미수신. 로봇 연결 또는 bag 재생 중인지 확인할 것')
            return
        self.get_logger().info(
            'in=%d  fix=%d  nofix=%d  parse_err=%d   last: hdop=%s sat=%s'
            % (self.n_in, self.n_fix, self.n_nofix, self.n_bad,
               self.last_hdop, self.last_sat))


def main():
    rclpy.init()
    node = GnssBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
