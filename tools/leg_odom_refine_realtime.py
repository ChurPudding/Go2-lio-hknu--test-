#!/usr/bin/env python3
# tools/leg_odom_refine.py — 실시간(실기) 구동용 핵심부
#
# bag 재생용과 로직은 동일하다. 유일한 실기 차이는 구독 QoS 다.
#   Unitree /utlidar/robot_odom 은 BEST_EFFORT 로 발행되므로,
#   기본 RELIABLE 로 구독하면 메시지가 조용히 씹힌다(silent rejection).
#   → 구독은 sensor QoS(BEST_EFFORT, depth 10)로 맞춘다.
#
#   - TF 발행 안 함 (map->odom 은 localization_stub 소유).
#   - z 축에는 k 를 곱하지 않는다.
#   - twist 는 보정 증분으로 다시 계산 (k 를 두 번 곱하지 않는다).
#   - kx_b = ky_b = 0 (기본)이면 기존 동작과 수치 동일.
#
# 실행:
#   cd ~/fastlio_ws/tools
#   source ~/unitree_ros2/setup_go2.sh
#   python3 leg_odom_refine.py --ros-args -r __ns:=/hknu \
#     -p kx_a:=1.23 -p ky_a:=1.23        # 실내면 1.1995

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry

try:
    import go2_calib
    K_DEFAULT = go2_calib.K_OUTDOOR
except Exception:
    K_DEFAULT = 1.23


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LegOdomRefine(Node):
    def __init__(self):
        super().__init__('leg_odom_refine')

        p = self.declare_parameter
        p('in_topic',   '/utlidar/robot_odom')
        p('out_topic',  'leg_odom')          # -r __ns:=/hknu → /hknu/leg_odom
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')

        p('enable_scale', True)
        p('kx_a', K_DEFAULT)
        p('kx_b', 0.0)
        p('ky_a', K_DEFAULT)
        p('ky_b', 0.0)
        p('k_min', 1.0)
        p('k_max', 1.5)
        p('input_timeout', 0.5)
        p('pos_var', 0.05)
        p('yaw_var', 0.02)

        g = lambda n: self.get_parameter(n).value
        self.in_topic   = g('in_topic')
        self.out_frame  = g('odom_frame')
        self.base_frame = g('base_frame')
        self.en_scale   = g('enable_scale')
        self.kx_a, self.kx_b = g('kx_a'), g('kx_b')
        self.ky_a, self.ky_b = g('ky_a'), g('ky_b')
        self.k_min, self.k_max = g('k_min'), g('k_max')
        self.timeout    = g('input_timeout')
        self.pos_var, self.yaw_var = g('pos_var'), g('yaw_var')

        # 상태
        self.prev_p = None
        self.prev_t = None
        self.p_corr = [0.0, 0.0, 0.0]
        self.k_clamped = 0

        # ── 실기 핵심: 구독은 BEST_EFFORT sensor QoS ──────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        # 출력은 하류(EKF/Nav2/팀원 A)와 맞춰야 한다. 기본은 RELIABLE depth 50.
        # 만약 하류가 BEST_EFFORT 로 구독하면 여기도 sensor_qos 로 바꿔라.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.pub = self.create_publisher(Odometry, g('out_topic'), pub_qos)
        self.create_subscription(Odometry, self.in_topic, self.on_odom, sensor_qos)

        self.get_logger().info(
            f"leg_odom_refine[RT]: {self.in_topic}(BEST_EFFORT) -> {g('out_topic')} "
            f"| scale={self.en_scale} kx_a={self.kx_a} ky_a={self.ky_a}")

        # 실기 워치독: 입력이 끊기면 경고 (silent rejection / 케이블 이슈 조기 발견)
        self.last_rx = None
        self.create_timer(1.0, self._watchdog)

    def _watchdog(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_rx is None:
            self.get_logger().warn(
                f"{self.in_topic} 수신 없음 — QoS(BEST_EFFORT) / 연결 확인")
        elif now - self.last_rx > 1.0:
            self.get_logger().warn(
                f"{self.in_topic} {now - self.last_rx:.1f}s 동안 끊김")

    def on_odom(self, msg):
        self.last_rx = self.get_clock().now().nanoseconds * 1e-9

        # 0단: 증분 추출
        pin = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_p is None:
            self.prev_p = [pin.x, pin.y, pin.z]
            self.prev_t = t
            return

        dt = t - self.prev_t
        if dt <= 0.0 or dt > self.timeout:
            self.prev_p = [pin.x, pin.y, pin.z]
            self.prev_t = t
            return

        dxw = pin.x - self.prev_p[0]
        dyw = pin.y - self.prev_p[1]
        dzw = pin.z - self.prev_p[2]

        yaw = yaw_from_quat(msg.pose.pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        dxb =  c * dxw + s * dyw
        dyb = -s * dxw + c * dyw
        dzb =  dzw

        # 3단: 축척 보정 (Δx, Δy 에만, z 는 그대로)
        if self.en_scale:
            speed = math.hypot(dxb, dyb) / dt
            kx = self.kx_a + self.kx_b * speed
            ky = self.ky_a + self.ky_b * speed
            kx_c = min(max(kx, self.k_min), self.k_max)
            ky_c = min(max(ky, self.k_min), self.k_max)
            if kx_c != kx or ky_c != ky:
                self.k_clamped += 1
            dxb *= kx_c
            dyb *= ky_c

        dxw2 = c * dxb - s * dyb
        dyw2 = s * dxb + c * dyb
        self.p_corr[0] += dxw2
        self.p_corr[1] += dyw2
        self.p_corr[2] += dzb

        # 발행
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.out_frame
        out.child_frame_id  = self.base_frame

        out.pose.pose.position.x = self.p_corr[0]
        out.pose.pose.position.y = self.p_corr[1]
        out.pose.pose.position.z = self.p_corr[2]
        out.pose.pose.orientation = msg.pose.pose.orientation

        out.twist.twist.linear.x = dxb / dt
        out.twist.twist.linear.y = dyb / dt
        out.twist.twist.linear.z = dzb / dt
        out.twist.twist.angular  = msg.twist.twist.angular

        for i, v in ((0, self.pos_var), (7, self.pos_var),
                     (14, self.pos_var), (35, self.yaw_var)):
            out.pose.covariance[i] = v

        self.pub.publish(out)

        self.prev_p = [pin.x, pin.y, pin.z]
        self.prev_t = t


def main():
    rclpy.init()
    node = LegOdomRefine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
