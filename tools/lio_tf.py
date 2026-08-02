#!/usr/bin/env python3
"""
lio_tf.py  --  /lio/base_pose 를 TF 로 발행한다 (camera_init -> base_link)

`go2_odom_tf.py` 를 대체한다
--------------------------
기존 `go2_odom_tf.py` 는 `/utlidar/robot_odom`(다리 운동학)으로 `odom -> base`
TF 를 냈다. 두 가지 문제가 있다.

  1. 다리 오도메트리는 표류한다. 실외 356 m 주행에서 수평 오차 RMS 10.2 m,
     최대 20.0 m (2026-07-31 실측). A* 가 이 위치를 믿으면 안 된다.
  2. LIO 와 동시에 쓰면 같은 프레임을 두 곳에서 발행해 TF 트리가 깨진다.

이 노드는 같은 역할을 **LIO 기반**으로 한다. `/indoor/base_pose` 는 `robot_pose.py`
가 LEVER 를 보정해 낸 몸통 위치이므로 지도와 같은 좌표계(camera_init)에 있다.

프레임 이름
----------
자식 프레임 기본값은 **`base_link`** 다. 기존 노드는 `base` 를 썼는데, Nav2 와
`robot_localization` 이 `base_link` 를 표준으로 가정하므로 바꿨다.
URDF 쪽 이름이 `base` 라면 `-p child_frame:=base` 로 맞추면 된다.

**타임스탬프는 입력 메시지의 것을 그대로 쓴다.** `now()` 를 쓰면 재생이나
지연 상황에서 extrapolation 오류가 난다.

health 연동
----------
`/lio/health` 가 false 가 되면 TF 발행을 멈춘다(기본). 위치를 못 믿는 상태에서
TF 를 계속 내면 A* 가 조용히 잘못된 경로를 만들기 때문이다. 멈추면 TF 조회가
실패하므로 하위 노드가 그 사실을 알 수 있다.

    /lio/base_pose ─┐
                    ├─> [lio_tf] ─> /tf : camera_init -> base_link
    /lio/health ────┘

사용
    python3 lio_tf.py
    python3 lio_tf.py --ros-args -p child_frame:=base -p ignore_health:=true
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class LioTf(Node):
    def __init__(self):
        super().__init__('lio_tf')

        self.declare_parameter('in_topic', '/indoor/base_pose')
        self.declare_parameter('parent_frame', 'indoor_map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('health_topic', '/indoor/health')
        self.declare_parameter('ignore_health', False)

        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.ignore_health = bool(self.get_parameter('ignore_health').value)

        self.healthy = True
        self.n = 0
        self.n_skip = 0

        self.br = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter('in_topic').value,
            self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Bool, self.get_parameter('health_topic').value,
                                 self.on_health, 10)
        self.create_timer(10.0, self.report)

        self.get_logger().info(
            'lio_tf: %s -> TF %s -> %s%s'
            % (self.get_parameter('in_topic').value, self.parent, self.child,
               '  (health 무시)' if self.ignore_health else ''))

    def on_health(self, m):
        if self.healthy and not m.data:
            self.get_logger().error('health=false — TF 발행을 멈춥니다')
        elif not self.healthy and m.data:
            self.get_logger().warn('health 회복 — TF 발행 재개')
        self.healthy = m.data

    def on_pose(self, m):
        if not self.healthy and not self.ignore_health:
            self.n_skip += 1
            return

        t = TransformStamped()
        t.header.stamp = m.header.stamp          # now() 를 쓰지 말 것
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self.br.sendTransform(t)
        self.n += 1

    def report(self):
        if self.n == 0 and self.n_skip == 0:
            self.get_logger().warn('입력 미수신 — robot_pose.py 가 떠 있는지 확인할 것')
            return
        self.get_logger().info('TF %d 회 발행, %d 회 건너뜀 (health=%s)'
                               % (self.n, self.n_skip, self.healthy))


def main():
    rclpy.init()
    n = LioTf()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
