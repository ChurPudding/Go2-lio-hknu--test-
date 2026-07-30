#!/usr/bin/env python3
"""
gnss_path.py  --  /gps/fix (NavSatFix) 를 국소 ENU 좌표로 바꿔 RViz 에 궤적으로 표시

gnss_bridge.py 뒤에 붙여 쓴다.

    /gnss --[gnss_bridge]--> /gps/fix --[gnss_path]--> /gps/path, /gps/pose

첫 유효 측위를 원점(0,0)으로 잡고 이후를 미터 단위 ENU(동-북-상)로 변환한다.
위경도 차이가 작을 때 쓰는 국소 평면 근사이며, 수백 m 범위에서는 오차가 무시할
수준이다(위도 1도 = 111320 m, 경도는 cos(위도) 배).

출력
----
  /gps/path  (nav_msgs/Path)          누적 궤적
  /gps/pose  (geometry_msgs/PoseStamped)  현재 위치
  frame_id 는 gps_local (기본값)

무효 측위(STATUS_NO_FIX)는 궤적에 넣지 않는다. 끊긴 구간은 직선으로 이어진다.

RViz 사용법
-----------
    ros2 run rviz2 rviz2

  1. Global Options > Fixed Frame 을 gps_local 로 입력
  2. Add > By topic > /gps/path > Path
  3. Add > By topic > /gps/pose > Pose   (현재 위치 화살표)

TF 가 없어도 Fixed Frame 이름이 Path 의 frame_id 와 같으면 그려진다.

사용
----
    python3 gnss_path.py
    python3 gnss_path.py --ros-args -p max_points:=5000
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

MLAT = 111320.0        # 위도 1도당 미터


class GnssPath(Node):
    def __init__(self):
        super().__init__('gnss_path')

        self.declare_parameter('in_topic', '/gps/fix')
        self.declare_parameter('frame_id', 'gps_local')
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('min_step', 0.0)   # 이 거리 미만 이동은 무시 [m]

        self.frame_id = self.get_parameter('frame_id').value
        self.max_points = int(self.get_parameter('max_points').value)
        self.min_step = float(self.get_parameter('min_step').value)

        self.lat0 = None
        self.lon0 = None
        self.mlon = None
        self.last = None          # 마지막 (x, y)
        self.dist = 0.0
        self.n_in = 0
        self.n_used = 0

        self.path = Path()
        self.path.header.frame_id = self.frame_id

        # Path 는 latched 로 두면 RViz 를 늦게 켜도 전체가 보인다
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub_path = self.create_publisher(Path, '/gps/path', latched)
        self.pub_pose = self.create_publisher(PoseStamped, '/gps/pose', 10)
        self.create_subscription(
            NavSatFix, self.get_parameter('in_topic').value,
            self.on_fix, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)

        self.get_logger().info(
            'gnss_path %s -> /gps/path, /gps/pose  (frame=%s)'
            % (self.get_parameter('in_topic').value, self.frame_id))

    # ------------------------------------------------------------------
    def on_fix(self, msg):
        self.n_in += 1

        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self.lat0 is None:
            self.lat0 = msg.latitude
            self.lon0 = msg.longitude
            self.mlon = MLAT * math.cos(math.radians(self.lat0))
            self.get_logger().info(
                '원점 고정: lat %.7f  lon %.7f' % (self.lat0, self.lon0))

        x = (msg.longitude - self.lon0) * self.mlon      # 동
        y = (msg.latitude - self.lat0) * MLAT            # 북

        if self.last is not None:
            step = math.hypot(x - self.last[0], y - self.last[1])
            if step < self.min_step:
                return
            self.dist += step
        self.last = (x, y)
        self.n_used += 1

        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_points:
            self.path.poses.pop(0)

        self.pub_path.publish(self.path)
        self.pub_pose.publish(ps)

    # ------------------------------------------------------------------
    def report(self):
        if self.n_in == 0:
            self.get_logger().warn(
                '/gps/fix 미수신. gnss_bridge.py 실행 여부와 bag 재생을 확인할 것')
            return
        if self.last is None:
            self.get_logger().info('in=%d, 유효 측위 없음' % self.n_in)
            return
        x, y = self.last
        self.get_logger().info(
            'in=%d used=%d  현재 (%.1f, %.1f) m  원점거리 %.1f m  총이동 %.1f m'
            % (self.n_in, self.n_used, x, y, math.hypot(x, y), self.dist))


def main():
    rclpy.init()
    node = GnssPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
