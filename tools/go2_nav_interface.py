#!/usr/bin/env python3
"""
go2_nav_interface.py  --  Nav2 가 필요로 하는 것을 한 노드에서 전부 낸다

    python3 go2_nav_interface.py
    python3 go2_nav_interface.py --ros-args -p map_yaml:=results/live_map.yaml

왜 하나로 합쳤나
--------------
전에는 static_transform_publisher 3개 + 릴레이 + 지도발행 + 점군변환으로
여섯 프로세스가 떠 있었다. 기동 순서, 프레임 이름, QoS 가 서로 얽혀서
무엇이 잘못됐는지 추적하기 어려웠다. 한 노드로 모으면 이렇게 된다.

    · 프레임 이름이 한 파일 안에 있다 (아래 상수)
    · TF 세 개가 같은 시각으로 나간다
    · 로그가 한 곳에 모인다
    · 켜고 끄기가 한 번

내보내는 것 (전부 Nav2 기본 이름)
-------------------------------
    /map    nav_msgs/OccupancyGrid   latched (transient_local)
    /odom   nav_msgs/Odometry        약 15 Hz
    /scan   sensor_msgs/LaserScan    약 15 Hz
    /tf     map -> odom -> base_link -> utlidar_lidar

받는 것
------
    /indoor/base_pose   우리 파이프라인의 위치 (run_indoor.sh)
    /utlidar/cloud      로봇 원시 점군

프레임 3단 구조
-------------
Nav2 는 map->odom 에 전역 보정량을, odom->base_link 에 국소 오도메트리를
담는 것을 전제한다. 우리는 LIO 하나뿐이고 이를 보정할 상위 계층이 없으므로
**map->odom 을 항등 변환**으로 둔다. 나중에 GPS 나 루프 클로저를 붙이면
그때 이 변환이 의미를 갖는다.

**팀원A 쪽에서 AMCL 과 slam_toolbox 를 끄셔야 합니다.** 둘 다 map->odom 을
발행하므로 동시에 켜면 TF 가 충돌해 로봇 위치가 튑니다.

위치 신뢰도
---------
`/indoor/base_pose` 의 covariance[0] 이 1e6 이면 위치를 믿을 수 없다는 뜻이다.
그때는 **/odom 과 TF 발행을 멈춘다.** 틀린 위치로 Nav2 가 주행하는 것을
막기 위해서다. Nav2 는 오도메트리 끊김으로 인식한다.

지도
---
부풀리지 않은 원본을 쓰십시오. Nav2 의 inflation_layer 가 로봇 크기만큼
여유를 알아서 둡니다. 우리가 한 번 더 부풀리면 이중 적용돼 통로가 막힙니다.
"""
import math
import os
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# ─── 프레임 이름 (Nav2 기본) ──────────────────────────────────────────
MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
LIDAR_FRAME = 'utlidar_lidar'      # /utlidar/cloud 의 frame_id

# ─── 외부 파라미터 (go2_calib 실측값) ─────────────────────────────────
# R_LB : 몸통 -> 라이다.  L1 은 164.9° 기울어 장착돼 있다.
import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from go2_calib import R_LB, LEVER   # 상수는 한 곳에서만 관리한다
R_BL = R_LB.T                              # 라이다 -> 몸통


def rot_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
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
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def load_map(yaml_path):
    """yaml + pgm 을 읽어 (occ[bool 2D], resolution, origin_x, origin_y)."""
    txt = open(yaml_path).read()

    def field(name, default=None):
        m = re.search(r'^\s*%s\s*:\s*(.+)$' % name, txt, re.M)
        return m.group(1).strip() if m else default

    image = field('image')
    res = float(field('resolution', '0.1'))
    nums = re.findall(r'-?\d+\.?\d*', field('origin', '[0,0,0]'))
    ox, oy = float(nums[0]), float(nums[1])

    pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    with open(pgm, 'rb') as f:
        assert f.readline().strip() == b'P5', 'P5 형식이 아닙니다'
        vals = []
        while len(vals) < 3:
            line = f.readline()
            if line.startswith(b'#'):
                continue
            vals += [int(v) for v in line.split()]
        w, h, maxval = vals[:3]
        px = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)

    # pgm 은 위아래가 뒤집혀 저장된다. OccupancyGrid 는 row 0 이 아래쪽이므로
    # 되돌린다. 값도 반대다 (pgm 0=장애물, OccupancyGrid 100=장애물).
    return (px < maxval / 2)[::-1], res, ox, oy


def cloud_xyz(msg):
    """PointCloud2 에서 x,y,z 만. x,y,z 가 앞 12바이트라고 본다."""
    n = msg.width * msg.height
    if n == 0 or msg.point_step < 12:
        return np.empty((0, 3))
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=n * msg.point_step)
    return raw.reshape(n, msg.point_step)[:, :12].copy() \
              .view(np.float32).reshape(n, 3).astype(np.float64)


class Go2NavInterface(Node):
    def __init__(self):
        super().__init__('go2_nav_interface')

        ws = os.path.expanduser('~/fastlio_ws')
        self.declare_parameter('in_pose', '/indoor/base_pose')
        self.declare_parameter('in_cloud', '/utlidar/cloud')
        self.declare_parameter('map_yaml', ws + '/results/indoor_map.yaml')
        self.declare_parameter('cov_threshold', 100.0)
        self.declare_parameter('publish_scan', True)
        # 스캔 변환 (몸통 프레임 기준 높이)
        self.declare_parameter('scan_min_height', -0.15)
        self.declare_parameter('scan_max_height', 0.60)
        self.declare_parameter('scan_range_min', 0.35)
        self.declare_parameter('scan_range_max', 12.0)
        self.declare_parameter('scan_bins', 720)          # 0.5° 간격

        g = lambda n: self.get_parameter(n).value
        self.cov_th = float(g('cov_threshold'))
        self.do_scan = bool(g('publish_scan'))
        self.hmin = float(g('scan_min_height'))
        self.hmax = float(g('scan_max_height'))
        self.rmin = float(g('scan_range_min'))
        self.rmax = float(g('scan_range_max'))
        self.bins = int(g('scan_bins'))

        self.ok = True
        self.n_odom = self.n_scan = self.n_skip = 0

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

        # ── 발행 ──────────────────────────────────────────────────────
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', latched)
        self.pub_scan = self.create_publisher(LaserScan, '/scan',
                                              qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)

        # ── 정적 TF 두 개 ─────────────────────────────────────────────
        self.publish_static_tf()

        # ── 지도 ──────────────────────────────────────────────────────
        self.map_msg = None
        try:
            self.load_and_publish_map(g('map_yaml'))
        except Exception as e:
            self.get_logger().error('지도 읽기 실패: %s' % e)
            self.get_logger().error('  경로: %s' % g('map_yaml'))

        # ── 구독 ──────────────────────────────────────────────────────
        self.create_subscription(Odometry, g('in_pose'),
                                 self.on_pose, qos_profile_sensor_data)
        if self.do_scan:
            self.create_subscription(PointCloud2, g('in_cloud'),
                                     self.on_cloud, qos_profile_sensor_data)
        self.create_timer(10.0, self.report)

        self.get_logger().info('go2_nav_interface 시작')
        self.get_logger().info('  받는 것 : %s, %s' % (g('in_pose'), g('in_cloud')))
        self.get_logger().info('  내는 것 : /odom, /map, /scan, /tf')
        self.get_logger().info('  TF      : %s -> %s -> %s -> %s'
                               % (MAP_FRAME, ODOM_FRAME, BASE_FRAME, LIDAR_FRAME))
        self.get_logger().warn('  ※ Nav2 쪽에서 AMCL 과 slam_toolbox 를 끄십시오')

    # ------------------------------------------------------------------
    def publish_static_tf(self):
        """map->odom (항등) 과 base_link->utlidar_lidar (외부 파라미터)."""
        now = self.get_clock().now().to_msg()
        out = []

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = MAP_FRAME
        t.child_frame_id = ODOM_FRAME
        t.transform.rotation.w = 1.0
        out.append(t)

        q = rot_to_quat(R_BL)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = BASE_FRAME
        t2.child_frame_id = LIDAR_FRAME
        t2.transform.translation.x = float(LEVER[0])
        t2.transform.translation.y = float(LEVER[1])
        t2.transform.translation.z = float(LEVER[2])
        t2.transform.rotation.x = float(q[0])
        t2.transform.rotation.y = float(q[1])
        t2.transform.rotation.z = float(q[2])
        t2.transform.rotation.w = float(q[3])
        out.append(t2)

        self.tf_static.sendTransform(out)
        self.get_logger().info('정적 TF 2개 발행 (%s->%s, %s->%s)'
                               % (MAP_FRAME, ODOM_FRAME, BASE_FRAME, LIDAR_FRAME))

    def load_and_publish_map(self, yaml_path):
        occ, res, ox, oy = load_map(yaml_path)
        h, w = occ.shape

        m = OccupancyGrid()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = res
        m.info.width = w
        m.info.height = h
        m.info.origin.position.x = ox
        m.info.origin.position.y = oy
        m.info.origin.orientation.w = 1.0
        m.data = np.where(occ, 100, 0).astype(np.int8).ravel().tolist()

        self.map_msg = m
        self.pub_map.publish(m)
        self.get_logger().info(
            '지도 /map  %d x %d, %.3f m/칸, 원점 (%.4f, %.4f), 장애물 %.1f%%'
            % (w, h, res, ox, oy, 100 * occ.mean()))
        self.get_logger().info('  %s' % os.path.basename(yaml_path))
        if 'inflated' in yaml_path:
            self.get_logger().warn(
                '  ※ 부풀린 지도입니다. Nav2 의 inflation 과 이중 적용됩니다.')

    # ------------------------------------------------------------------
    def on_pose(self, m):
        """위치를 /odom 과 TF(odom->base_link) 로 낸다."""
        if m.pose.covariance[0] > self.cov_th:
            if self.ok:
                self.get_logger().error(
                    '위치 신뢰 불가 (covariance=%.0f) — /odom 과 TF 를 멈춥니다'
                    % m.pose.covariance[0])
                self.ok = False
            self.n_skip += 1
            return
        if not self.ok:
            self.get_logger().warn('위치 신뢰도 회복 — 발행 재개')
            self.ok = True

        o = Odometry()
        o.header.stamp = m.header.stamp        # 원본 시각 유지
        o.header.frame_id = ODOM_FRAME
        o.child_frame_id = BASE_FRAME
        o.pose = m.pose
        o.twist = m.twist
        self.pub_odom.publish(o)

        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_FRAME
        p = m.pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)
        self.n_odom += 1

    def on_cloud(self, msg):
        """3D 점군을 2D LaserScan 으로. local costmap 의 obstacle_layer 용."""
        P = cloud_xyz(msg)
        if len(P) == 0:
            return
        P = P[np.isfinite(P).all(1)]

        # 라이다 프레임 -> 몸통 프레임
        B = P @ R_BL.T + LEVER

        m = (B[:, 2] > self.hmin) & (B[:, 2] < self.hmax)
        B = B[m]
        if len(B) == 0:
            return

        rng = np.hypot(B[:, 0], B[:, 1])
        k = (rng > self.rmin) & (rng < self.rmax)
        B, rng = B[k], rng[k]
        if len(B) == 0:
            return

        ang = np.arctan2(B[:, 1], B[:, 0])
        inc = 2 * math.pi / self.bins
        idx = ((ang + math.pi) / inc).astype(int).clip(0, self.bins - 1)

        # 각 방향에서 가장 가까운 점만 남긴다
        out = np.full(self.bins, np.inf)
        np.minimum.at(out, idx, rng)

        s = LaserScan()
        s.header.stamp = msg.header.stamp
        s.header.frame_id = BASE_FRAME     # 이미 몸통 프레임으로 옮겼다
        s.angle_min = -math.pi
        s.angle_max = math.pi - inc
        s.angle_increment = inc
        s.time_increment = 0.0
        s.scan_time = 1.0 / 15.0
        s.range_min = self.rmin
        s.range_max = self.rmax
        s.ranges = out.astype(np.float32).tolist()
        self.pub_scan.publish(s)
        self.n_scan += 1

    def report(self):
        if self.n_odom == 0 and self.n_skip == 0:
            self.get_logger().warn(
                '위치 미수신 — run_indoor.sh 가 떠 있는지 확인할 것')
            return
        self.get_logger().info('odom %d, scan %d, 건너뜀 %d  (ok=%s)'
                               % (self.n_odom, self.n_scan, self.n_skip, self.ok))
        # 지도는 latched 지만 늦게 붙는 구독자를 위해 가끔 다시 낸다
        if self.map_msg is not None:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_map.publish(self.map_msg)


def main():
    rclpy.init()
    n = Go2NavInterface()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
