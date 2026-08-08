#!/usr/bin/env python3
"""
FAST-LIO 포인트 클라우드를 CSV로 저장 + voxel 다운샘플링(최적화).

  /cloud_registered (월드 좌표 정합 결과) 또는
  /utlidar/cloud    (라이다 원본) 을 구독해,
  지정한 프레임 수만큼 모아 CSV로 저장한다.

사용:
    source ~/setup_go2.sh
    python3 cloud_to_csv.py                      # 기본: /cloud_registered, 1프레임
    python3 cloud_to_csv.py --topic /utlidar/cloud
    python3 cloud_to_csv.py --frames 10 --voxel 0.05   # 10프레임 누적, 5cm voxel
    python3 cloud_to_csv.py --voxel 0               # voxel 끄기(원본 그대로)

옵션:
    --topic   구독 토픽 (기본 /cloud_registered)
    --frames  누적할 프레임 수 (기본 1)
    --voxel   voxel 크기(m). 0이면 다운샘플 안 함 (기본 0.05)
    --out     출력 파일명 (기본 cloud_YYYYmmdd_HHMMSS.csv)
"""
import argparse, struct, csv, datetime, sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def parse_cloud(msg):
    """PointCloud2 -> Nx4 numpy (x, y, z, intensity)."""
    step = msg.point_step
    n = msg.width * msg.height
    # 필드 offset 찾기
    off = {f.name: f.offset for f in msg.fields}
    ox, oy, oz = off['x'], off['y'], off['z']
    oi = off.get('intensity', None)
    buf = bytes(msg.data)
    pts = np.empty((n, 4), dtype=np.float32)
    for i in range(n):
        b = i * step
        pts[i, 0] = struct.unpack_from('<f', buf, b + ox)[0]
        pts[i, 1] = struct.unpack_from('<f', buf, b + oy)[0]
        pts[i, 2] = struct.unpack_from('<f', buf, b + oz)[0]
        pts[i, 3] = struct.unpack_from('<f', buf, b + oi)[0] if oi is not None else 0.0
    # NaN/inf 제거
    mask = np.isfinite(pts).all(axis=1)
    return pts[mask]


def voxel_downsample(pts, size):
    """간단한 voxel grid 다운샘플: 각 voxel의 평균점 하나만 남긴다."""
    if size <= 0 or len(pts) == 0:
        return pts
    keys = np.floor(pts[:, :3] / size).astype(np.int64)
    # 각 voxel의 대표(첫 등장) 인덱스만
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


class Collector(Node):
    def __init__(self, topic, frames):
        super().__init__('cloud_to_csv')
        self.target = frames
        self.buf = []
        self.sub = self.create_subscription(PointCloud2, topic, self.cb, 10)
        self.get_logger().info(f'{topic} 구독, {frames}프레임 수집 대기...')

    def cb(self, msg):
        pts = parse_cloud(msg)
        self.buf.append(pts)
        self.get_logger().info(f'  프레임 {len(self.buf)}/{self.target}: {len(pts)}점')
        if len(self.buf) >= self.target:
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/cloud_registered')
    ap.add_argument('--frames', type=int, default=1)
    ap.add_argument('--voxel', type=float, default=0.05)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    rclpy.init()
    node = Collector(args.topic, args.frames)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if not node.buf:
        print('수집된 점이 없습니다. 토픽이 발행 중인지 확인하세요.')
        sys.exit(1)

    allpts = np.vstack(node.buf)
    raw_n = len(allpts)
    allpts = voxel_downsample(allpts, args.voxel)

    out = args.out or f"cloud_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'z', 'intensity'])
        w.writerows(allpts.tolist())

    print(f'\n원본 {raw_n}점 -> 저장 {len(allpts)}점 (voxel={args.voxel}m)')
    print(f'저장 완료: {out}')


if __name__ == '__main__':
    main()
