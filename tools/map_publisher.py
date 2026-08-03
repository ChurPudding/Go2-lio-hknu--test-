#!/usr/bin/env python3
"""
map_publisher.py  --  지도 파일을 /indoor/map 토픽으로 발행한다

    python3 map_publisher.py
    python3 map_publisher.py --ros-args -p yaml:=results/indoor_map.yaml

nav2_map_server 와 같은 일을 하지만 의존성이 없고 lifecycle 관리도 필요 없다.
(nav2_map_server 를 쓰시려면 그쪽도 됩니다. 아래 '다른 방법' 참고)

**QoS 가 중요하다.** 지도는 한 번만 발행하고 붙잡아 두는(latched) 것이라
`TRANSIENT_LOCAL` + `RELIABLE` 로 낸다. 구독하는 쪽도 같게 맞춰야 하며,
기본값(VOLATILE)으로 구독하면 **아무것도 받지 못한다.**

값 규약 (nav_msgs/OccupancyGrid 표준)
    data[row * width + col]
      0   자유
      100 장애물
      -1  미지 (이 지도에는 없음)
    row 0 이 아래쪽. pgm 과 달리 뒤집혀 있지 않다.

좌표 변환
    col = (x - origin.x) / resolution
    row = (y - origin.y) / resolution
  /indoor/base_pose 와 같은 원점이므로 그대로 넣으면 된다.

다른 방법 (nav2 를 이미 쓰신다면)
    sudo apt install -y ros-humble-nav2-map-server
    ros2 run nav2_map_server map_server --ros-args \
      -p yaml_filename:=$HOME/fastlio_ws/results/indoor_map_inflated.yaml
    ros2 lifecycle set /map_server activate      # 다른 터미널에서
"""
import os
import re
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid


def load_map(yaml_path):
    """yaml + pgm 을 읽어 (occ[bool 2D], resolution, origin_x, origin_y) 반환."""
    txt = open(yaml_path).read()

    def field(name, default=None):
        m = re.search(r'^\s*%s\s*:\s*(.+)$' % name, txt, re.M)
        return m.group(1).strip() if m else default

    image = field('image')
    res = float(field('resolution', '0.1'))
    org = field('origin', '[0,0,0]')
    ox, oy = [float(v) for v in re.findall(r'-?\d+\.?\d*', org)[:2]]

    pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    with open(pgm, 'rb') as f:
        assert f.readline().strip() == b'P5', 'P5 형식이 아닙니다'
        # 주석(#)을 건너뛰며 폭·높이·최댓값을 읽는다
        vals = []
        while len(vals) < 3:
            line = f.readline()
            if line.startswith(b'#'):
                continue
            vals += [int(v) for v in line.split()]
        w, h, maxval = vals[:3]
        px = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)

    # pgm 은 위아래가 뒤집혀 저장돼 있다. OccupancyGrid 는 row 0 이 아래쪽이므로 되돌린다.
    # 값도 반대다: pgm 은 0 이 장애물, OccupancyGrid 는 100 이 장애물.
    occ = (px < maxval / 2)[::-1]
    return occ, res, ox, oy


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter(
            'yaml', os.path.expanduser(
                '~/fastlio_ws/results/indoor_map_inflated.yaml'))
        self.declare_parameter('topic', '/indoor/map')
        self.declare_parameter('frame_id', 'indoor_map')
        self.declare_parameter('period', 0.0)   # 0 이면 한 번만 (latched 로 충분)

        y = self.get_parameter('yaml').value
        if not os.path.isfile(y):
            self.get_logger().error('지도 파일 없음: %s' % y)
            raise SystemExit(1)

        occ, res, ox, oy = load_map(y)
        h, w = occ.shape

        self.msg = OccupancyGrid()
        self.msg.header.frame_id = self.get_parameter('frame_id').value
        self.msg.info.resolution = res
        self.msg.info.width = w
        self.msg.info.height = h
        self.msg.info.origin.position.x = ox
        self.msg.info.origin.position.y = oy
        self.msg.info.origin.orientation.w = 1.0
        self.msg.data = np.where(occ, 100, 0).astype(np.int8).ravel().tolist()

        # latched: 늦게 붙는 구독자도 받을 수 있다
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('topic').value, qos)
        self.publish()

        p = float(self.get_parameter('period').value)
        if p > 0:
            self.create_timer(p, self.publish)

        self.get_logger().info(
            '%s  %d x %d, %.3f m/칸, 원점 (%.4f, %.4f), 장애물 %d 칸 (%.1f%%)'
            % (self.get_parameter('topic').value, w, h, res, ox, oy,
               int(occ.sum()), 100 * occ.mean()))
        self.get_logger().info(
            '구독 시 QoS 를 TRANSIENT_LOCAL 로 맞출 것. 기본값이면 못 받습니다.')

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    try:
        n = MapPublisher()
    except SystemExit:
        rclpy.shutdown(); return
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
