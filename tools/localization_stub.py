#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localization_stub.py — 위치 추정 인터페이스 스텁 (1단계)

무엇을 하는가
-------------
다리 오도메트리의 축척 오차만 보정해 `map` 프레임을 만든다. 그 이상은 하지
않는다. 나중에 GTSAM 이 이 노드를 그대로 대체하며, 그때 하류(경로계획)는
코드를 고치지 않는다. 이 노드의 존재 이유가 바로 그 인터페이스 고정이다.

    1단계 (이 노드)  map -> odom = 축척 오차만 상쇄
    2단계 (앵커링)   map 을 GPS 원점 + 진북에 고정
    3단계 (GTSAM)    map -> odom 에 팩터그래프 해를 실시간으로 발행

왜 map -> odom 을 매번 다시 계산하는가
--------------------------------------
축척 오차는 거리에 비례해 커지므로 고정 변환으로는 없앨 수 없다. 출발점에서만
맞고 멀어질수록 (k-1) x 거리 만큼 벌어진다.

회전은 축척의 영향을 받지 않는다(다리 오도메트리의 yaw 는 IMU 적분이며 보폭
추정과 무관하다). 따라서 원하는 결과는

    p_map = k * p_odom,   q_map = q_odom

이고, map -> base_link = (map -> odom) * (odom -> base_link) 에서 회전이 같으므로
map -> odom 은 순수 평행이동이 된다.

    t_map_odom = p_map - p_odom = (k - 1) * p_odom      (회전 = 항등)

이 형태는 GTSAM 단계와 동일하다. GTSAM 은 같은 자리에 더 좋은 값을 넣을 뿐이다.

TF 소유권
---------
    map --[이 노드]--> odom --[Go2]--> base_link

/tf 는 여러 노드가 함께 쓰도록 설계된 토픽이다. 여럿이 발행하는 것 자체는
정상이며 오류가 아니다. 문제가 되는 것은 **같은 부모-자식 쌍**을 둘이
동시에 발행할 때다. 두 값이 번갈아 들어와 위치가 튀는데, 에러가 나지
않으므로 원인을 찾기 어렵다.

    Go2 가 odom->base_link, 이 노드가 map->odom        정상
    Go2 와 이 노드가 둘 다 odom->base_link             튐
    go2_nav_interface.py 와 이 노드가 둘 다 map->odom  튐

연결선마다 주인은 하나다.

    map -> odom          실외: 이 노드 / 실내: go2_nav_interface.py
                         (동시 실행 금지)
    odom -> base_link    Go2 내부

tf_guard 가 켜져 있으면(기본) 발행 시작 전 2초간 /tf 를 엿들어 같은 연결선의
주인이 이미 있는지 확인하고, 있으면 그 TF 를 발행하지 않는다. 확인:

    ros2 run tf2_tools view_frames

z 축
----
z 는 축척을 곱하지 않고 그대로 통과시킨다. Go2 의 z 는 고도가 아니라 몸통
높이이며, 축척 k 는 수평 보폭 추정에서 나온 값이라 z 에 적용할 근거가 없다.

이 노드가 하지 않는 것
----------------------
  - 절대 위치·방위 고정 (map 원점은 부팅 지점, +x 는 부팅 시 yaw)
    -> 위경도 웨이포인트를 map 좌표로 바꿀 수 없다. 2단계에서 해결한다.
  - IMU yaw 표류 보정 (2~5 deg/min). heading_core.py 가 담당할 몫이다.
  - 루프 폐합, GPS 융합, 지도 작성

토픽
----
    구독  /utlidar/robot_odom          Go2 가 정한 이름. 바꿀 수 없다
    발행  /hknu/robot_odom             nav_msgs/Odometry
          /hknu/robot_pose             geometry_msgs/PoseWithCovarianceStamped
          /tf                          map -> odom

/hknu 는 팀 네임스페이스다. 노드 안에서는 상대 이름(robot_odom)으로 선언하고
실행할 때 __ns 로 씌운다. 이름을 문자열로 박지 않는 이유는 나중에 여러 대를
띄우거나 이름을 바꿀 때 코드를 고치지 않기 위함이다.

  ** /tf 와 /tf_static 은 반드시 네임스페이스 밖으로 되돌려야 한다. **

노드를 네임스페이스에 넣으면 TF 토픽까지 /hknu/tf 로 딸려 들어간다. tf2
리스너는 전역 /tf 만 보므로 RViz, view_frames, nav2 가 전부 TF 를 못 찾는다.
에러가 나지 않고 조용히 실패하므로 특히 조심할 것.

실내와의 관계
-------------
이 노드는 **실외 전용**이다. 실내는 Point-LIO 기반 go2_nav_interface.py 가
담당한다. 둘 다 map->odom 을 발행하므로 동시에 띄우면 안 된다.

    실내   run_indoor.sh   -> go2_nav_interface.py   (Nav2 기본 이름)
    실외   run_outdoor.sh  -> 이 노드                (/hknu 네임스페이스)

토픽 이름은 실내·실외를 구분하지 않는다. 하류(경로계획)가 어느 쪽인지 알
필요가 없어야 하기 때문이다. 구분되는 것은 실행 스크립트와 노드 이름이다.

사용
----
    source ~/unitree_ros2/setup_go2.sh

    python3 localization_stub.py --ros-args \
        -r __ns:=/hknu -r /tf:=/tf -r /tf_static:=/tf_static

    python3 localization_stub.py --ros-args \
        -r __ns:=/hknu -r /tf:=/tf -r /tf_static:=/tf_static \
        -p k:=1.2007 -p k_verified:=true

    # bag 재생으로 확인
    srcoff
    ros2 bag play <bag> -r 0.5
    ros2 topic echo /hknu/robot_pose --once
    ros2 run tf2_tools view_frames
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from go2_calib import K_OUTDOOR as K_DEFAULT
except ImportError:
    K_DEFAULT = 1.23
    print("[warn] go2_calib.py 를 찾지 못해 k=1.23 을 직접 씁니다.")


class LocalizationStub(Node):

    def __init__(self):
        super().__init__('localization_stub')

        p = self.declare_parameter

        # 출력은 상대 이름으로 선언한다. 네임스페이스를 실행 시 씌우면
        # /hknu/robot_odom, /hknu/robot_pose 가 된다.
        #   --ros-args -r __ns:=/hknu -r /tf:=/tf -r /tf_static:=/tf_static
        # 입력은 Go2 가 정한 이름이라 절대 경로로 둔다.
        p('in_topic', '/utlidar/robot_odom')
        p('out_topic', 'robot_odom')
        p('out_pose_topic', 'robot_pose')

        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')

        # 축척 계수. go2_calib.K_OUTDOOR 에서 가져온다.
        # 상수는 한 곳에서만 관리한다 — 여기에 숫자를 적지 말 것.
        p('k', K_DEFAULT)
        p('k_verified', False)

        # TF 충돌 감시: 발행 전에 /tf 를 엿들어 같은 연결선의 주인이
        # 이미 있는지 확인한다. 자세한 내용은 위 docstring 참조.
        p('tf_guard', True)
        p('tf_guard_sec', 2.0)

        p('publish_map_odom_tf', True)
        # Go2 가 odom -> base_link 를 이미 방송하면 반드시 False 로 둘 것.
        # 켜면 base_link 의 부모가 둘이 되어 TF 트리가 깨진다.
        p('publish_odom_base_tf', False)

        # 자세 공분산. 스텁 단계에서는 실측 근거가 없으므로 크게 잡는다.
        # 하류가 이 값을 믿고 융합하지 않도록 하는 것이 목적이다.
        p('pos_sigma', 1.0)          # m
        p('yaw_sigma_deg', 10.0)     # deg

        g = lambda n: self.get_parameter(n).value

        self.k = float(g('k'))
        self.k_verified = bool(g('k_verified'))
        self.map_frame = str(g('map_frame'))
        self.odom_frame = str(g('odom_frame'))
        self.base_frame = str(g('base_frame'))
        self.pub_map_odom = bool(g('publish_map_odom_tf'))
        self.pub_odom_base = bool(g('publish_odom_base_tf'))
        self.tf_guard = bool(g('tf_guard'))
        self.tf_guard_sec = float(g('tf_guard_sec'))
        self.pos_var = float(g('pos_sigma')) ** 2
        self.yaw_var = math.radians(float(g('yaw_sigma_deg'))) ** 2

        self.n_in = 0
        self.last_raw = None       # 원시 (x, y)
        self.raw_dist = 0.0        # 원시 누적 이동거리

        latching = QoSProfile(depth=10)
        latching.reliability = ReliabilityPolicy.RELIABLE

        self.pub_odom = self.create_publisher(Odometry, g('out_topic'), latching)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, g('out_pose_topic'), latching)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Odometry, g('in_topic'),
                                 self.on_odom, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)

        self.get_logger().info(
            f"localization_stub  {g('in_topic')} -> "
            f"{self.pub_odom.topic_name}  k={self.k:.4f}")
        self.get_logger().info(
            f"  TF: {self.map_frame} -> {self.odom_frame} "
            f"{'발행' if self.pub_map_odom else '끔'}, "
            f"{self.odom_frame} -> {self.base_frame} "
            f"{'발행' if self.pub_odom_base else '끔 (Go2 담당)'}")

        if not self.k_verified:
            self.get_logger().warn(
                f"k={self.k:.4f} 는 go2_calib.K_OUTDOOR 기본값입니다. "
                "odom_scale_check.py 로 5m 직진 3회를 재고 "
                "k_verified:=true 로 바꿔 주세요.")

        if self.tf_guard:
            self.check_tf_owners()

    # ------------------------------------------------------------------
    def check_tf_owners(self):
        """이미 같은 연결선을 쏘는 노드가 있는지 확인한다.

        /tf 는 여러 노드가 함께 쓰는 토픽이므로 그 자체는 충돌이 아니다.
        문제는 **같은 부모-자식 쌍**을 둘이 동시에 발행할 때다. 그때는
        두 값이 번갈아 들어와 위치가 튄다. 에러가 나지 않으므로 원인을
        찾기 어렵다.

        아직 우리가 아무것도 발행하지 않은 시점이므로, 여기서 보이는
        연결선은 전부 남의 것이다.
        """
        seen = set()

        def on_tf(msg):
            for tr in msg.transforms:
                seen.add((tr.header.frame_id.lstrip('/'),
                          tr.child_frame_id.lstrip('/')))

        subs = [self.create_subscription(TFMessage, t, on_tf, 10)
                for t in ('/tf', '/tf_static')]

        self.get_logger().info(f"TF 감시 {self.tf_guard_sec:.0f}초...")
        t0 = time.time()
        while time.time() - t0 < self.tf_guard_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
        for s in subs:
            self.destroy_subscription(s)

        mo = (self.map_frame, self.odom_frame)
        ob = (self.odom_frame, self.base_frame)

        if seen:
            self.get_logger().info(
                f"  이미 발행 중인 연결선 {len(seen)}개: "
                + ", ".join(f"{a}->{b}" for a, b in sorted(seen)))
        else:
            self.get_logger().info("  발행 중인 TF 없음")

        if self.pub_map_odom and mo in seen:
            self.pub_map_odom = False
            self.get_logger().error(
                f"** {mo[0]}->{mo[1]} 를 이미 다른 노드가 발행 중입니다. **")
            self.get_logger().error(
                "   충돌을 막기 위해 이 노드는 해당 TF 를 발행하지 않습니다.")
            self.get_logger().error(
                "   go2_nav_interface.py 나 slam_toolbox, amcl 이 떠 있는지 "
                "확인하고 하나만 남겨 주세요.")

        if self.pub_odom_base and ob in seen:
            self.pub_odom_base = False
            self.get_logger().error(
                f"** {ob[0]}->{ob[1]} 는 Go2 가 이미 발행 중입니다. **")
            self.get_logger().error(
                "   publish_odom_base_tf 를 끈 채로 두세요.")
        elif not self.pub_odom_base and ob not in seen:
            self.get_logger().warn(
                f"{ob[0]}->{ob[1]} 를 아무도 발행하지 않습니다. "
                "publish_odom_base_tf:=true 가 필요할 수 있습니다.")

    # ------------------------------------------------------------------
    def on_odom(self, msg):
        self.n_in += 1

        pin = msg.pose.pose.position
        x, y, z = pin.x, pin.y, pin.z

        if self.last_raw is not None:
            self.raw_dist += math.hypot(x - self.last_raw[0], y - self.last_raw[1])
        self.last_raw = (x, y)

        # --- 보정된 자세 -------------------------------------------------
        # 수평만 k 를 곱한다. 회전은 그대로.
        mx, my = self.k * x, self.k * y

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.base_frame

        out.pose.pose.position.x = mx
        out.pose.pose.position.y = my
        out.pose.pose.position.z = z          # 몸통 높이. 축척 미적용
        out.pose.pose.orientation = msg.pose.pose.orientation

        cov = [0.0] * 36
        cov[0] = self.pos_var        # x
        cov[7] = self.pos_var        # y
        cov[14] = self.pos_var       # z
        cov[21] = self.yaw_var       # roll
        cov[28] = self.yaw_var       # pitch
        cov[35] = self.yaw_var       # yaw
        out.pose.covariance = cov

        # 속도는 base_link 기준. 병진만 k 를 곱한다.
        out.twist.twist.linear.x = self.k * msg.twist.twist.linear.x
        out.twist.twist.linear.y = self.k * msg.twist.twist.linear.y
        out.twist.twist.linear.z = msg.twist.twist.linear.z
        out.twist.twist.angular = msg.twist.twist.angular
        out.twist.covariance = cov

        self.pub_odom.publish(out)

        ps = PoseWithCovarianceStamped()
        ps.header = out.header
        ps.pose.pose = out.pose.pose
        ps.pose.covariance = cov
        self.pub_pose.publish(ps)

        # --- TF: map -> odom (순수 평행이동) -------------------------------
        if self.pub_map_odom:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.odom_frame
            t.transform.translation.x = (self.k - 1.0) * x
            t.transform.translation.y = (self.k - 1.0) * y
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0       # 항등 회전
            self.tf.sendTransform(t)

        # --- TF: odom -> base_link (Go2 가 안 할 때만) ----------------------
        if self.pub_odom_base:
            t2 = TransformStamped()
            t2.header.stamp = msg.header.stamp
            t2.header.frame_id = self.odom_frame
            t2.child_frame_id = self.base_frame
            t2.transform.translation.x = x
            t2.transform.translation.y = y
            t2.transform.translation.z = z
            t2.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(t2)

    # ------------------------------------------------------------------
    def report(self):
        if self.n_in == 0:
            self.get_logger().warn(
                f"{self.get_parameter('in_topic').value} 미수신. "
                "로봇 연결 또는 bag 재생을 확인해 주세요.")
            return
        x, y = self.last_raw
        self.get_logger().info(
            f"in={self.n_in}  원시 ({x:.2f}, {y:.2f}) m  "
            f"보정 ({self.k*x:.2f}, {self.k*y:.2f}) m  "
            f"원시 이동 {self.raw_dist:.1f} m  "
            f"보정 이동 {self.k*self.raw_dist:.1f} m")


def main():
    rclpy.init()
    node = LocalizationStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
