#!/usr/bin/env python3
"""
body_imu_tick2.py -- /lowstate 몸통 IMU -> /body_imu_tick   (2026-08-05 rev2)

v1 과의 차이 — 왜 고쳤는가
--------------------------
v1 은 기준점을 now() - tick/1000 으로 잡았다. 실시간에서는 맞지만
bag 재생에서는 now() 가 "지금 시각"이라 offset 이 통째로 밀린다.

  실측: IMU stamp 1785938173 (22:56, 현재)
        LiDAR stamp 1785918060 (17:21, bag 녹화 시각)
        -> 5시간 35분 차이. Point-LIO 가 짝을 못 맞춰 초기화조차 안 됨.

v2 는 /utlidar/imu 의 header 를 기준으로 삼는다.
같은 bag 안에 들어 있으므로 실시간이든 재생이든, 배속이 얼마든 항상 맞는다.

  offset = median(utlidar_imu.stamp - lowstate.tick/1000)

두 토픽은 같은 로봇에서 거의 동시에 나오므로, 도착 순서 잡음만 중앙값으로
걸러내면 기준점이 정확해진다.

주의
----
출력은 몸통(base) 프레임이다. config 에 extrinsic 을 반드시 넣을 것.
  R_BL 로 시도 -> 실패 (2026-08-05 C1/C2)
  R_LB 로 시도 -> 이번 실험

사용:
    python3 body_imu_tick2.py
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from unitree_go.msg import LowState
from sensor_msgs.msg import Imu


class BodyImuTick2(Node):
    CALIB_N = 300          # 기준점 표본 수

    def __init__(self):
        super().__init__('body_imu_tick2')
        self.declare_parameter('out_topic', '/body_imu_tick')
        self.declare_parameter('ref_topic', '/utlidar/imu')   # 시각 기준
        self.declare_parameter('frame_id', 'base')

        self.frame_id = self.get_parameter('frame_id').value

        self.ref_stamp = None   # /utlidar/imu 의 최신 stamp [s]
        self._buf = []
        self.offset = None
        self.n = 0

        pub_qos = QoSProfile(depth=200,
                             history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            Imu, self.get_parameter('out_topic').value, pub_qos)

        self.create_subscription(
            Imu, self.get_parameter('ref_topic').value,
            self.on_ref, qos_profile_sensor_data)
        self.create_subscription(
            LowState, '/lowstate', self.cb, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

        log = self.get_logger()
        log.info('body_imu_tick2 -> %s  (시각 기준 = %s)'
                 % (self.get_parameter('out_topic').value,
                    self.get_parameter('ref_topic').value))
        log.warn('config 에 extrinsic_R 을 확인할 것. 몸통 프레임 그대로 나간다.')

    # ----------------------------------------------------------
    def on_ref(self, msg):
        self.ref_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def cb(self, msg):
        if self.ref_stamp is None:
            return                      # 기준 토픽이 아직 안 옴

        t_tick = int(msg.tick) / 1000.0

        if self.offset is None:
            self._buf.append(self.ref_stamp - t_tick)
            if len(self._buf) >= self.CALIB_N:
                self.offset = float(np.median(self._buf))
                spread = float(np.max(self._buf) - np.min(self._buf))
                self.get_logger().info(
                    '[기준점] offset=%.6f  표본 %d개  편차 %.1f ms'
                    % (self.offset, len(self._buf), spread * 1000))
                self.get_logger().info(
                    '  -> 첫 출력 stamp ≈ %.3f  (기준 %.3f)'
                    % (t_tick + self.offset, self.ref_stamp))
            return

        stamp_sec = t_tick + self.offset

        out = Imu()
        out.header.stamp.sec = int(stamp_sec)
        out.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
        out.header.frame_id = self.frame_id

        g = msg.imu_state.gyroscope
        a = msg.imu_state.accelerometer
        out.angular_velocity.x = float(g[0])
        out.angular_velocity.y = float(g[1])
        out.angular_velocity.z = float(g[2])
        out.linear_acceleration.x = float(a[0])
        out.linear_acceleration.y = float(a[1])
        out.linear_acceleration.z = float(a[2])
        out.orientation_covariance[0] = -1.0   # orientation 미사용 표시

        self.pub.publish(out)
        self.n += 1

    def report(self):
        if self.ref_stamp is None:
            self.get_logger().warn('%s 미수신 — bag 재생이 시작됐는지 확인'
                                   % self.get_parameter('ref_topic').value)
        elif self.offset is None:
            self.get_logger().warn('기준점 잡는 중... (%d/%d)'
                                   % (len(self._buf), self.CALIB_N))
        else:
            self.get_logger().info('published=%d' % self.n)


def main():
    rclpy.init()
    node = BodyImuTick2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
