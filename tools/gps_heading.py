#!/usr/bin/env python3
"""gps_heading.py -- HeadingEstimator 를 ROS 토픽에 연결하는 껍데기.

계산은 전부 `heading_core.py` 에 있다. 이 파일은 토픽을 받아 코어에 넣고
결과를 발행할 뿐이다. 따라서 `tools/verify_heading.py` 로 bag 검증한 코드와
실기에서 도는 코드가 동일하다.

발행
----
    ~/heading        std_msgs/Float32   절대 방위 [deg]. ENU (동쪽 0, 반시계 +)
    ~/heading_info   std_msgs/String    상태 JSON

구독
----
    /gps/fix         sensor_msgs/NavSatFix        (gnss_bridge.py 출력)
    /lowstate        unitree_go/LowState          IMU
    /sportmodestate  unitree_go/SportModeState    몸통 속도

사용
----
    python3 gps_heading.py --ros-args -p baseline:=3.0 -p tau:=60.0

주의
----
QoS 는 전부 센서 프로파일(BEST_EFFORT)로 구독한다. 발행자가 RELIABLE 이어도
호환되지만 반대는 연결 자체가 안 된다. 토픽은 보이는데 콜백이 안 불리면
거의 항상 QoS 문제다.
"""
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from unitree_go.msg import LowState, SportModeState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from heading_core import HeadingEstimator  # noqa: E402


class GpsHeadingNode(Node):

    def __init__(self):
        super().__init__('gps_heading')

        p = self.declare_parameter
        p('gps_topic', '/gps/fix')
        p('lowstate_topic', '/lowstate')
        p('sport_topic', '/sportmodestate')
        p('out_topic', '~/heading')
        p('out_info_topic', '~/heading_info')

        p('baseline', 3.0)
        p('max_span', 20.0)
        p('bow_ratio', 0.20)
        p('min_speed', 0.20)
        p('max_lateral', 0.40)
        p('beta_correct', True)
        p('tau', 60.0)
        p('init_tau', 4.0)
        p('init_count', 8)
        p('outlier_deg', 40.0)

        p('rate', 20.0)          # Hz  방위 발행 주기
        p('info_rate', 5.0)      # Hz  상태 발행 주기
        p('gps_timeout', 5.0)    # s   이보다 오래 GPS 없으면 경고
        p('obs_timeout', 30.0)   # s   이보다 오래 관측 없으면 경고

        g = lambda n: self.get_parameter(n).value

        self.est = HeadingEstimator(
            baseline=float(g('baseline')),
            max_span=float(g('max_span')),
            bow_ratio=float(g('bow_ratio')),
            min_speed=float(g('min_speed')),
            max_lateral=float(g('max_lateral')),
            beta_correct=bool(g('beta_correct')),
            tau=float(g('tau')),
            init_tau=float(g('init_tau')),
            init_count=int(g('init_count')),
            outlier_deg=float(g('outlier_deg')),
        )
        self.gps_timeout = float(g('gps_timeout'))
        self.obs_timeout = float(g('obs_timeout'))
        self.warned_converged = False

        self.pub = self.create_publisher(Float32, g('out_topic'), 10)
        self.pub_info = self.create_publisher(String, g('out_info_topic'), 10)

        self.create_subscription(NavSatFix, g('gps_topic'),
                                 self.on_gps, qos_profile_sensor_data)
        self.create_subscription(LowState, g('lowstate_topic'),
                                 self.on_lowstate, qos_profile_sensor_data)
        self.create_subscription(SportModeState, g('sport_topic'),
                                 self.on_sport, qos_profile_sensor_data)

        self.create_timer(1.0 / float(g('rate')), self.tick)
        self.create_timer(1.0 / float(g('info_rate')), self.publish_info)
        self.create_timer(5.0, self.watchdog)

        self.get_logger().info(
            f"gps_heading: 기선 {self.est.L:.1f} m, 시정수 {self.est.tau:.0f} s, "
            f"β 보정 {'켬' if self.est.use_beta else '끔'}")

    # ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_lowstate(self, msg):
        q = msg.imu_state.quaternion          # [w, x, y, z]
        yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                         1 - 2 * (q[2] ** 2 + q[3] ** 2))
        self.est.push_imu(self.now(), yaw)

    def on_sport(self, msg):
        self.est.push_velocity(self.now(),
                               float(msg.velocity[0]), float(msg.velocity[1]))

    def on_gps(self, msg):
        before = self.est.n_obs
        self.est.push_gps(self.now(), msg.latitude, msg.longitude)
        if before == 0 and self.est.n_obs == 1:
            self.get_logger().info(
                f'초기 오프셋 {math.degrees(self.est.offset):+.1f}° 설정')
        if not self.warned_converged and self.est.converged:
            self.warned_converged = True
            self.get_logger().info(
                f'수렴 완료. 관측 {self.est.n_obs}개, '
                f'방위 {math.degrees(self.est.heading()):+.1f}°')

    # ------------------------------------------------------------------
    def tick(self):
        h = self.est.heading()
        if h is None:
            return
        m = Float32()
        m.data = float(math.degrees(h))
        self.pub.publish(m)

    def publish_info(self):
        s = String()
        s.data = json.dumps(self.est.status(self.now()), ensure_ascii=False)
        self.pub_info.publish(s)

    def watchdog(self):
        t = self.now()
        e = self.est
        if e.offset is None:
            self.get_logger().warn(
                f'아직 오프셋 없음 — GPS {len(e.gps)}개, '
                f'IMU {"수신" if e.yaws else "없음"}, '
                f'속도 {"수신" if e.vels else "없음"}, '
                f'기각 {e.n_reject} {e.reject_reason}')
            return
        if e.last_gps_t and t - e.last_gps_t > self.gps_timeout:
            self.get_logger().warn(f'GPS 끊김 {t - e.last_gps_t:.1f} s')
        elif e.last_obs_t and t - e.last_obs_t > self.obs_timeout:
            self.get_logger().warn(
                f'{t - e.last_obs_t:.0f} s 동안 방위 관측 없음 — '
                '표류가 쌓이는 중 (정지·회전 구간)')


def main():
    rclpy.init()
    node = GpsHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
