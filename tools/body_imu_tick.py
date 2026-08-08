#!/usr/bin/env python3
"""
body_imu_tick.py -- /lowstate 몸통 IMU -> /body_imu_tick  (2026-08-05)

기존 body_imu_bridge.py 와의 차이
---------------------------------
body_imu_bridge.py 는 header.stamp 를 now() 로 찍는다. 그러면 센서 측정
시각이 아니라 "이 PC 가 받은 시각"이 기록되므로, 네트워크 지연(수 ms,
일정하지 않음)이 그대로 오차가 되고 LiDAR de-skewing 이 어긋난다.

/lowstate 에는 header 가 없지만 tick 필드가 있다.
2026-08-05 실측: tick = 1 kHz 밀리초 카운터 (1001.1 Hz / 1000.3 Hz, 2회 재현).
간격 중앙값 2 (=500Hz 발행). 최대값이 튀는 것은 네트워크 도착 지연이지
tick 자체의 문제가 아니다.

따라서 tick 으로 시각을 복원하면 도착 지연과 무관하게 실제 측정 시각을
쓸 수 있다.

기준점 잡기
-----------
tick 은 로봇 부팅 후 경과 ms 라 Unix 시각이 아니다. 시작 직후 N 개 샘플로
  offset = median(도착시각 - tick/1000)
를 구해 고정한다. 이후에는 stamp = tick/1000 + offset.

중앙값을 쓰는 이유: 평균은 한 번의 큰 지연에 끌려가지만 중앙값은 견딘다.
그리고 지연은 항상 양수이므로, 표본이 많을수록 최소값 쪽이 참값에 가깝다.
여기서는 하위 10% 분위수를 쓴다.

주의
----
이 노드의 출력은 몸통(base) 프레임이다. LiDAR 프레임이 아니다.
Point-LIO / FAST-LIO config 에 extrinsic_R = R_BL 을 반드시 넣을 것.
(l1_imu_fix.py 는 노드 안에서 회전을 끝내므로 단위행렬이었다. 여기는 다르다.)

사용:
    python3 body_imu_tick.py
    python3 body_imu_tick.py --ros-args -p out_topic:=/body_imu_tick
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from unitree_go.msg import LowState
from sensor_msgs.msg import Imu


class BodyImuTick(Node):
    CALIB_N = 500          # 기준점 추정에 쓸 샘플 수 (500Hz 기준 약 1초)

    def __init__(self):
        super().__init__('body_imu_tick')
        self.declare_parameter('out_topic', '/body_imu_tick')
        self.declare_parameter('frame_id', 'base')
        self.declare_parameter('use_tick', True)   # False 면 now() (구버전 비교용)

        self.frame_id = self.get_parameter('frame_id').value
        self.use_tick = bool(self.get_parameter('use_tick').value)

        self._buf = []          # (도착시각, tick) 표본
        self.offset = None      # tick/1000 + offset = unix 시각
        self.n = 0
        self.tick_prev = None
        self.wrap = 0           # tick 이 uint32 를 넘어 되감길 경우

        pub_qos = QoSProfile(depth=200,
                             history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            Imu, self.get_parameter('out_topic').value, pub_qos)
        self.create_subscription(
            LowState, '/lowstate', self.cb, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

        log = self.get_logger()
        log.info('body_imu_tick -> %s (frame=%s, use_tick=%s)'
                 % (self.get_parameter('out_topic').value,
                    self.frame_id, self.use_tick))
        log.warn('config 에 extrinsic_R = R_BL 을 넣었는지 확인할 것. '
                 '이 노드는 몸통 프레임 그대로 내보낸다.')

    # ----------------------------------------------------------
    def cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        tick = int(msg.tick)

        # tick 되감김 처리 (uint32 약 49.7일이면 넘지 않지만 방어적으로)
        if self.tick_prev is not None and tick < self.tick_prev - 2**31:
            self.wrap += 1
        self.tick_prev = tick
        t_sec = (tick + self.wrap * 2**32) / 1000.0

        if self.use_tick:
            if self.offset is None:
                self._buf.append(now - t_sec)
                if len(self._buf) >= self.CALIB_N:
                    # 하위 10% 분위수 = 지연이 가장 적었던 구간
                    self.offset = float(np.quantile(self._buf, 0.10))
                    spread = float(np.max(self._buf) - np.min(self._buf))
                    self.get_logger().info(
                        '[기준점] offset=%.6f  표본 %d개  지연 편차 %.1f ms'
                        % (self.offset, len(self._buf), spread * 1000))
                return                      # 보정 전에는 발행하지 않는다
            stamp_sec = t_sec + self.offset
        else:
            stamp_sec = now

        out = Imu()
        out.header.stamp.sec = int(stamp_sec)
        out.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
        out.header.frame_id = self.frame_id

        q = msg.imu_state.quaternion            # [w, x, y, z]
        g = msg.imu_state.gyroscope             # rad/s
        a = msg.imu_state.accelerometer         # m/s^2

        out.orientation.w = float(q[0])
        out.orientation.x = float(q[1])
        out.orientation.y = float(q[2])
        out.orientation.z = float(q[3])
        out.angular_velocity.x = float(g[0])
        out.angular_velocity.y = float(g[1])
        out.angular_velocity.z = float(g[2])
        out.linear_acceleration.x = float(a[0])
        out.linear_acceleration.y = float(a[1])
        out.linear_acceleration.z = float(a[2])
        # orientation 은 채우긴 했으나 LIO 는 참조하지 않는다.
        # 잘못 쓰이는 것을 막으려면 아래 줄의 주석을 풀 것.
        # out.orientation_covariance[0] = -1.0

        self.pub.publish(out)
        self.n += 1

    def report(self):
        if self.offset is None:
            self.get_logger().warn('기준점 잡는 중... (%d/%d)'
                                   % (len(self._buf), self.CALIB_N))
        else:
            self.get_logger().info('published=%d' % self.n)


def main():
    rclpy.init()
    node = BodyImuTick()
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
