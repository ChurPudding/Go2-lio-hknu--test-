#!/usr/bin/env python3
"""
Go2 odom -> base TF 발행 노드
============================
/utlidar/robot_odom (nav_msgs/Odometry) 에서 로봇의 실시간 위치와 자세를 받아,
odom -> base 좌표 변환(TF)으로 발행합니다.

이 노드가 있으면 로봇이 실제로 걸어갈 때, RViz의 로봇 모델도 좌표평면 위를
따라 움직입니다. (고정 static_transform 은 로봇을 제자리에 묶어두지만,
이 노드는 실시간 위치를 반영합니다.)

좌표계 구성:
  odom --(이 노드, 실시간)--> base --(정적 TF, 고정)--> 라이다/센서

주의:
  이 노드를 쓰기 전에, 기존에 켜 둔
  'odom -> base' static_transform_publisher 는 반드시 종료(Ctrl+C)하세요.
  같은 odom->base 를 두 곳에서 발행하면 충돌하여 로봇이 떨립니다.

실행:
    python3 go2_odom_tf.py

종료: Ctrl+C
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# 발행할 부모/자식 프레임 이름
PARENT_FRAME = "odom"
CHILD_FRAME = "base"


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__("go2_odom_tf")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            Odometry, "/utlidar/robot_odom", self.callback, qos
        )
        self.br = TransformBroadcaster(self)

        self._count = 0
        self.get_logger().info(
            "odom -> base TF 발행 시작. /utlidar/robot_odom 수신 대기 중..."
        )

    def callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = PARENT_FRAME
        t.child_frame_id = CHILD_FRAME

        # 위치 (position)
        p = msg.pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z

        # 자세 (orientation, 쿼터니언)
        q = msg.pose.pose.orientation
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        self.br.sendTransform(t)

        self._count += 1
        if self._count % 100 == 0:
            self.get_logger().info(
                f"TF 발행 중... 현재 위치 x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}"
            )


def main():
    rclpy.init()
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
