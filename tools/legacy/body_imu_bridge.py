#!/usr/bin/env python3
"""
/lowstate (unitree_go/msg/LowState) 안의 몸통 IMU를
sensor_msgs/Imu 타입 /body_imu 토픽으로 변환 발행하는 브리지.

라이다 내장 IMU(/utlidar/imu)가 약 19도 기울어 장착돼 있어
extrinsic이 안 맞는 문제를 피하기 위해, 수평인 몸통 IMU를 쓴다.

  /lowstate.imu_state.quaternion    [w,x,y,z] -> orientation
  /lowstate.imu_state.gyroscope     [x,y,z]   -> angular_velocity
  /lowstate.imu_state.accelerometer [x,y,z]   -> linear_acceleration

사용:
    source ~/setup_go2.sh
    python3 body_imu_bridge.py

발행 주파수는 /lowstate 주파수를 따른다 (보통 500Hz 내외).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_go.msg import LowState
from sensor_msgs.msg import Imu


class BodyImuBridge(Node):
    def __init__(self):
        super().__init__('body_imu_bridge')

        # 센서 데이터용 QoS (best effort)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.pub = self.create_publisher(Imu, '/body_imu', qos)
        self.sub = self.create_subscription(
            LowState, '/lowstate', self.cb, qos)
        self.n = 0
        self.get_logger().info('/lowstate -> /body_imu 브리지 시작')

    def cb(self, msg):
        imu = Imu()
        # 타임스탬프: 현재 시각으로 찍음 (lowstate 자체 stamp가 없으므로)
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'body_imu'

        q = msg.imu_state.quaternion       # [w, x, y, z]
        g = msg.imu_state.gyroscope        # [x, y, z] rad/s
        a = msg.imu_state.accelerometer    # [x, y, z] m/s^2

        imu.orientation.w = float(q[0])
        imu.orientation.x = float(q[1])
        imu.orientation.y = float(q[2])
        imu.orientation.z = float(q[3])

        imu.angular_velocity.x = float(g[0])
        imu.angular_velocity.y = float(g[1])
        imu.angular_velocity.z = float(g[2])

        imu.linear_acceleration.x = float(a[0])
        imu.linear_acceleration.y = float(a[1])
        imu.linear_acceleration.z = float(a[2])

        self.pub.publish(imu)

        self.n += 1
        if self.n % 500 == 0:
            self.get_logger().info(
                f'  발행 {self.n}회  accel=({a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f})')


def main():
    rclpy.init()
    node = BodyImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
