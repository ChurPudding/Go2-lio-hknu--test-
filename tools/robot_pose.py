#!/usr/bin/env python3
"""
robot_pose.py -- Point-LIO 출력에서 로봇 몸통(base_link)의 실시간 위치 벡터

왜 필요한가
-----------
/aft_mapped_to_init 은 LiDAR(=IMU) 원점의 자세다. 로봇 몸통 위치가 아니다.
두 가지 보정이 필요하다.

  1) 자세 : LiDAR 프레임은 몸통에 대해 165.6도 회전해 있다 (R_LB)
            그냥 쓰면 heading 이 약 128도 틀어진다
  2) 위치 : LiDAR 는 몸통 중심에서 정면으로 0.322 m 앞에 있다 (lever arm)
            제자리 회전만 해도 LiDAR 는 반지름 0.32 m 원호를 그린다

    p_base(map) = p_lidar(map) - R_ML @ R_LB @ r

출력
----
  /lio/base_pose  (nav_msgs/Odometry)   몸통 위치·자세, frame=camera_init
  콘솔          x, y, z, heading

검증
----
로봇을 제자리에서 한 바퀴 돌린다.
  - /aft_mapped_to_init 의 x,y 는 반지름 0.32 m 원을 그린다
  - /lio/base_pose 의 x,y 는 거의 제자리에 머문다  <- 이게 맞으면 성공
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# 본체(base_link) -> L1/LiDAR 회전. 자이로 Kabsch 정렬, 설명력 98.0%
R_LB = np.array([
    [+0.523029, -0.838576, +0.152420],
    [-0.810712, -0.544668, -0.214668],
    [+0.263034, -0.011292, -0.964721],
])

# base_link 원점 -> LiDAR 원점, base_link 프레임 표현 [m]
#   x,y : 창 분할 최소자승으로 추정 (잔차 0.238 -> 0.143 m, 40% 개선)
#   z   : 이 데이터로는 관측 불가(몸통 roll/pitch 변화 부족).
#         지면까지 거리 0.355~0.364 m 와 몸통 높이 0.308 m 로부터 추정한 근사값.
#         정밀도가 필요하면 실측할 것.
LEVER = np.array([0.322, 0.005, 0.050])

FWD_L = R_LB @ np.array([1.0, 0.0, 0.0])   # 로봇 정면을 LiDAR 프레임으로


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class RobotPose(Node):
    def __init__(self):
        super().__init__('lio_base_pose')
        self.declare_parameter('in_topic', '/aft_mapped_to_init')
        self.declare_parameter('out_topic', '/lio/base_pose')
        self.declare_parameter('print_hz', 2.0)
        self.declare_parameter('apply_lever', True)

        self.last = None
        self.pub = self.create_publisher(
            Odometry, self.get_parameter('out_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('in_topic').value, self.on_odom, 10)

        hz = float(self.get_parameter('print_hz').value)
        if hz > 0:
            self.create_timer(1.0 / hz, self.report)
        self.get_logger().info('lio_base_pose ready')

    def on_odom(self, m):
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        R_ML = quat_to_R(o.x, o.y, o.z, o.w)      # LiDAR -> map
        R_MB = R_ML @ R_LB                        # base_link -> map

        p_lidar = np.array([p.x, p.y, p.z])
        if self.get_parameter('apply_lever').value:
            p_base = p_lidar - R_MB @ LEVER
        else:
            p_base = p_lidar

        f = R_MB @ np.array([1.0, 0.0, 0.0])      # 로봇 정면 (map)
        heading = math.atan2(f[1], f[0])
        self.last = (p_base, heading, m.header.stamp)

        out = Odometry()
        out.header = m.header
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = float(p_base[0])
        out.pose.pose.position.y = float(p_base[1])
        out.pose.pose.position.z = float(p_base[2])
        qx, qy, qz, qw = R_to_quat(R_MB)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist = m.twist
        self.pub.publish(out)

    def report(self):
        if self.last is None:
            self.get_logger().warn('no odometry yet')
            return
        p, h, _ = self.last
        self.get_logger().info(
            'base_link  x=%+7.3f  y=%+7.3f  z=%+7.3f  heading=%+7.1f deg'
            % (p[0], p[1], p[2], math.degrees(h)))


def main():
    rclpy.init()
    node = RobotPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
