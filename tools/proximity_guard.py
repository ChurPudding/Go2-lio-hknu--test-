#!/usr/bin/env python3
"""
proximity_guard.py  --  앞이 막히면 정지 신호를 낸다 (최소 안전장치)

왜 필요한가
----------
실주행 시험 중 로봇이 벽으로 걸어가는 것을 막는다. 리모컨만 믿기에는 사람이
알아채고 손이 가기까지 0.5~1초가 걸리는데, 0.5 m/s 로 걸으면 그 사이 50 cm 를
간다.

**팀원B 의 회피 알고리즘을 대신하는 것이 아니다.** 이것은 "피하는" 것이 아니라
"멈추는" 것뿐이다. 회피가 완성되면 그대로 두어도 되고(이중 안전장치), 빼도 된다.

설계 원칙 — LIO 에 의존하지 않는다
--------------------------------
입력은 **`/utlidar/cloud` (원시 점군)** 이다. `/cloud_registered` 가 아니다.

  원시 점군은 라이다 프레임이라 위치추정이 필요 없다.
  LIO 가 발산해도(실측: 실외 27 km) 이 노드는 정상 동작한다.

즉 위치가 완전히 틀어진 상황에서도 "앞에 벽이 있다"는 사실은 여전히 맞다.
안전장치는 가장 아래 계층에 두어야 한다.

판정
----
로봇 앞쪽 부채꼴(기본 ±50°) 안에서, 지면보다 높고 로봇 폭 안에 있는 점 중
가장 가까운 것이 stop_dist 보다 가까우면 정지.

  - 지면 제거: UP 방향 성분이 ground_min 보다 낮은 점은 지면으로 본다
  - 자기 다리 제거: min_range 이내 점은 버린다
  - 잡음 제거: 조건을 만족하는 점이 min_points 개 이상일 때만 인정한다

LIO health 와 달리 **래치하지 않는다.** 장애물이 치워지면 바로 풀린다.
물리적 상태이므로 자동 회복이 맞다.

출력
----
  /indoor/safe          std_msgs/Bool     false = 앞이 막힘, 정지할 것
  /indoor/obstacle      std_msgs/String   JSON. 거리·방향·점 수

팀원A 쪽 사용
-------------
    if not safe:
        stop()

`/indoor/base_pose` 의 covariance 검사와 **둘 다** 넣으시면 됩니다.
전자는 "위치를 모른다", 후자는 "앞이 막혔다" 로 원인이 다릅니다.

사용
----
    python3 proximity_guard.py
    python3 proximity_guard.py --ros-args -p stop_dist:=0.8 -p sector_deg:=60
"""
import json
import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String

# 라이다 프레임에서 본 '위' 방향과 '앞' 방향.
# go2_calib 이 있으면 그쪽 값을 쓰고, 없으면 아래 실측값을 쓴다.
UP_L = np.array([1.66, -1.90, -9.48])
R_LB = np.array([
    [+0.523029, -0.838576, +0.152420],
    [-0.810712, -0.544668, -0.214668],
    [+0.263034, -0.011292, -0.964721],
])
try:
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import go2_calib
    R_LB = np.array(getattr(go2_calib, 'R_LB', R_LB))
    UP_L = np.array(getattr(go2_calib, 'EXPECTED_REST_ACC', UP_L))
except Exception:
    pass

UP_L = UP_L / np.linalg.norm(UP_L)
FWD_L = R_LB @ np.array([1.0, 0.0, 0.0])          # 로봇 정면을 라이다 프레임으로
FWD_L = FWD_L - UP_L * (FWD_L @ UP_L)             # 수평면에 투영
FWD_L = FWD_L / np.linalg.norm(FWD_L)
LEFT_L = np.cross(UP_L, FWD_L)


def cloud_xyz(msg):
    """PointCloud2 에서 x,y,z 만 뽑는다. 필드 순서는 x,y,z 가 앞에 있다고 본다."""
    n = msg.width * msg.height
    if n == 0 or msg.point_step < 12:
        return np.empty((0, 3))
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=n * msg.point_step)
    xyz = raw.reshape(n, msg.point_step)[:, :12].copy()
    return xyz.view(np.float32).reshape(n, 3).astype(np.float64)


class ProximityGuard(Node):
    def __init__(self):
        super().__init__('proximity_guard')

        self.declare_parameter('in_topic', '/utlidar/cloud')
        self.declare_parameter('stop_dist', 0.70)      # 이보다 가까우면 정지 [m]
        self.declare_parameter('clear_dist', 0.90)     # 이보다 멀어져야 해제 [m]
        self.declare_parameter('sector_deg', 50.0)     # 앞쪽 부채꼴 반각 [deg]
        self.declare_parameter('half_width', 0.35)     # 로봇 반폭 [m]
        self.declare_parameter('ground_min', 0.12)     # 지면 위 이 높이부터 장애물 [m]
        self.declare_parameter('ceil_max', 0.90)       # 이보다 높으면 무시 [m]
        self.declare_parameter('min_range', 0.30)      # 자기 다리 제거 [m]
        self.declare_parameter('self_radius', 0.55)    # 이 반경 안의 낮은 점은 자기 다리
        self.declare_parameter('self_height', 0.40)    # 이 높이 아래만 자기 것으로 본다
        self.declare_parameter('persist', 3)           # 연속 N 프레임 걸려야 정지
        self.declare_parameter('min_points', 5)        # 잡음 제거
        self.declare_parameter('timeout', 1.0)         # 점군 끊기면 정지 [s]
        self.declare_parameter('out_topic', '/indoor/safe')
        self.declare_parameter('out_info_topic', '/indoor/obstacle')

        g = lambda n: self.get_parameter(n).value
        self.stop_d = float(g('stop_dist'))
        self.clear_d = float(g('clear_dist'))
        self.cos_sec = math.cos(math.radians(float(g('sector_deg'))))
        self.half_w = float(g('half_width'))
        self.gmin = float(g('ground_min'))
        self.cmax = float(g('ceil_max'))
        self.rmin = float(g('min_range'))
        self.self_r = float(g('self_radius'))
        self.self_h = float(g('self_height'))
        self.persist = int(g('persist'))
        self.hits = 0
        self.npts = int(g('min_points'))
        self.timeout = float(g('timeout'))

        self.safe = True
        self.last = None
        self.info = {}
        self.n_in = 0

        self.pub = self.create_publisher(Bool, g('out_topic'), 10)
        self.pub_info = self.create_publisher(String, g('out_info_topic'), 10)
        self.create_subscription(PointCloud2, g('in_topic'),
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            'proximity_guard: %s -> %s   정지 %.2f m, 해제 %.2f m, 부채꼴 ±%.0f°'
            % (g('in_topic'), g('out_topic'), self.stop_d, self.clear_d,
               float(g('sector_deg'))))

    # ------------------------------------------------------------------
    def on_cloud(self, msg):
        self.n_in += 1
        self.last = self.get_clock().now().nanoseconds * 1e-9

        P = cloud_xyz(msg)
        if len(P) == 0:
            return
        P = P[np.isfinite(P).all(1)]

        rng = np.linalg.norm(P, axis=1)
        P = P[rng > self.rmin]
        if len(P) == 0:
            self.update(None, 0, 0.0)
            return

        # 지면 높이를 매 스캔에서 추정한다.
        # h 는 라이다 원점 기준이라 그대로 쓰면 라이다 장착 높이(약 0.32 m)만큼
        # 기준이 떠서 낮은 장애물을 놓친다. 점의 대부분이 지면이므로
        # 하위 20 백분위를 지면으로 잡고 그로부터의 높이를 쓴다.
        h_raw = P @ UP_L
        ground_h = float(np.percentile(h_raw, 20))
        h = h_raw - ground_h              # 지면 기준 높이
        f = P @ FWD_L                     # 앞쪽 거리
        s = P @ LEFT_L                    # 좌우 거리

        m = ((h > self.gmin) & (h < self.cmax)   # 지면·천장 제거
             & (f > 0)                            # 앞쪽만
             & (np.abs(s) < self.half_w))         # 로봇 폭 안
        hor = np.hypot(f, s)
        # 자기 다리 제거: 가깝고 낮은 점은 로봇 자신이다.
        # L1 은 비반복 스캔이라 프레임마다 다리에 맞는 정도가 달라
        # 이 필터가 없으면 한 프레임 걸러 오탐이 난다.
        m &= ~((hor < self.self_r) & (h < self.self_h))
        # 부채꼴 제한 (수평 거리 기준)
        m &= (hor > 1e-6)
        m[m] &= (f[m] / hor[m]) > self.cos_sec

        Q = P[m]
        if len(Q) < self.npts:
            self.update(None, len(Q), 0.0)
            return

        d = np.hypot(Q @ FWD_L, Q @ LEFT_L)
        i = int(np.argmin(d))
        ang = math.degrees(math.atan2(Q[i] @ LEFT_L, Q[i] @ FWD_L))
        self.update(float(d[i]), len(Q), ang)

    # ------------------------------------------------------------------
    def update(self, dist, n, ang):
        """이력현상(hysteresis): 정지는 stop_d, 해제는 clear_d 로 떨림을 막는다."""
        if dist is None:
            blocked = False
        elif self.safe:
            blocked = dist <= self.stop_d
        else:
            blocked = dist <= self.clear_d
        # 연속 persist 프레임 걸려야 정지한다. 단발 잡음을 거른다.
        self.hits = self.hits + 1 if blocked else 0
        new = not (self.hits >= self.persist)

        if self.safe and not new:
            self.get_logger().error('장애물과 너무 가깝습니다 — %.2f m, %+.0f°, %d 점'
                                    % (dist, ang, n))
        elif not self.safe and new:
            self.get_logger().warn('장애물에서 벗어났습니다')
        self.safe = new
        self.info = {'safe': new,
                     'dist': round(dist, 3) if dist is not None else None,
                     'angle_deg': round(ang, 1), 'points': n}

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last is not None and now - self.last > self.timeout:
            if self.safe:
                self.get_logger().error('점군 끊김 %.1fs — 앞을 볼 수 없습니다'
                                        % (now - self.last))
            self.safe = False
            self.info = {'safe': False, 'reason': 'cloud_timeout'}
        self.pub.publish(Bool(data=self.safe))
        self.pub_info.publish(String(data=json.dumps(self.info, ensure_ascii=False)))

    def report(self):
        if self.n_in == 0:
            self.get_logger().warn('점군 미수신 — 로봇 연결을 확인할 것')
            return
        self.get_logger().info('safe=%s  %s  (수신 %d)'
                               % (self.safe, self.info, self.n_in))


def main():
    rclpy.init()
    n = ProximityGuard()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
