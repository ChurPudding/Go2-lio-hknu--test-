#!/usr/bin/env python3
# tools/leg_odom_refine.py — 보정 토픽 발행 핵심부 (0단 Δ추출 + 3단 축척)
#
# /utlidar/robot_odom (누적 절대 위치)를 프레임 간 증분으로 바꾼 뒤,
# body 프레임 증분(Δx, Δy)에만 축척 k를 곱해 /hknu/leg_odom 으로 재발행한다.
#
#   - TF 는 발행하지 않는다 (map->odom 소유권은 localization_stub).
#   - z 축에는 k 를 곱하지 않는다 (Go2 z 는 몸통 높이, k 는 수평 보폭값).
#   - twist 는 보정된 증분으로 다시 계산한다 (k 를 두 번 곱하지 않기 위해).
#   - kx_b = ky_b = 0 (기본)이면 기존 localization_stub 출력과 수치 동일.
#
# 실행:  ros2 run ... leg_odom_refine.py --ros-args -r __ns:=/hknu \
#          -p kx_a:=1.23 -p ky_a:=1.23

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

try:
    import go2_calib
    K_DEFAULT = go2_calib.K_OUTDOOR   # 실외 1.23 / 실내는 실행 인자로 K_INDOOR
except Exception:
    K_DEFAULT = 1.23


def yaw_from_quat(q):
    # z 축 회전만 (yaw). roll/pitch 는 무시.
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LegOdomRefine(Node):
    def __init__(self):
        super().__init__('leg_odom_refine')

        p = self.declare_parameter
        p('in_topic',   '/utlidar/robot_odom')
        p('out_topic',  'leg_odom')          # 상대 이름 → -r __ns:=/hknu 로 /hknu/leg_odom
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')

        p('enable_scale', True)
        p('kx_a', K_DEFAULT)                 # k_x = kx_a + kx_b*speed
        p('kx_b', 0.0)
        p('ky_a', K_DEFAULT)                 # k_y = ky_a + ky_b*speed
        p('ky_b', 0.0)
        p('k_min', 1.0)
        p('k_max', 1.5)
        p('input_timeout', 0.5)              # dt 가 이보다 크면 그 프레임 버림 (bag 점프 대비)

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

        # 누적 상태
        self.prev_p   = None      # 직전 world 절대 위치 (x, y, z)
        self.prev_t   = None      # 직전 stamp (sec)
        self.p_corr   = [0.0, 0.0, 0.0]   # 보정된 누적 위치
        self.k_clamped = 0

        self.pub = self.create_publisher(Odometry, g('out_topic'), 50)
        self.create_subscription(Odometry, self.in_topic, self.on_odom, 50)

        self.get_logger().info(
            f"leg_odom_refine: {self.in_topic} -> {g('out_topic')} "
            f"| scale={self.en_scale} kx_a={self.kx_a} ky_a={self.ky_a}")

    def on_odom(self, msg):
        # ── 0단: 프레임 간 증분 추출 ────────────────────────────────
        pin = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_p is None:
            self.prev_p = [pin.x, pin.y, pin.z]
            self.prev_t = t
            return

        dt = t - self.prev_t
        if dt <= 0.0 or dt > self.timeout:      # 시간 역전/점프 프레임은 버림
            self.prev_p = [pin.x, pin.y, pin.z]
            self.prev_t = t
            return

        dxw = pin.x - self.prev_p[0]
        dyw = pin.y - self.prev_p[1]
        dzw = pin.z - self.prev_p[2]

        # world 증분 → body 증분 (yaw 역회전)
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        dxb =  c * dxw + s * dyw
        dyb = -s * dxw + c * dyw
        dzb =  dzw

        # ── 3단: 축척 보정 — body 증분(Δx, Δy)에만 적용, z 는 그대로 ──
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

        # body 증분 → world 로 되돌려 누적
        dxw2 = c * dxb - s * dyb
        dyw2 = s * dxb + c * dyb
        self.p_corr[0] += dxw2
        self.p_corr[1] += dyw2
        self.p_corr[2] += dzb          # z 는 축척 없이 통과

        # ── 발행 ────────────────────────────────────────────────
        out = Odometry()
        out.header.stamp = msg.header.stamp      # 입력 stamp 유지 (재생 시각 아님)
        out.header.frame_id = self.out_frame
        out.child_frame_id  = self.base_frame

        out.pose.pose.position.x = self.p_corr[0]
        out.pose.pose.position.y = self.p_corr[1]
        out.pose.pose.position.z = self.p_corr[2]
        out.pose.pose.orientation = msg.pose.pose.orientation   # 자세는 원본 그대로

        # twist: 보정된 body 증분 / dt  (k 를 다시 곱하지 않는다)
        out.twist.twist.linear.x = dxb / dt
        out.twist.twist.linear.y = dyb / dt
        out.twist.twist.linear.z = dzb / dt
        out.twist.twist.angular  = msg.twist.twist.angular

        # 공분산 대각만
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
