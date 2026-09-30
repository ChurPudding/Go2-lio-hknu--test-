#!/usr/bin/env python3
# =============================================================================
# timesync_lio_logger.py   (rev3 - 다중 토픽/다중 타입 도착 로거)
# -----------------------------------------------------------------------------
# 목적 : bag play 로 실코드(LIO) 돌리는 동안, 아래를 하나의 벽시계(도착)축으로 기록.
#   - 라이다 IMU (raw /utlidar/imu)          [sensor_msgs/Imu]
#   - 수정 IMU  (l1_imu_fix 출력, LIO 입력)   [sensor_msgs/Imu]
#   - 몸통 IMU  (/lowstate)                   [unitree_go/msg/LowState]
#   - 오도메트리 여러 개 (LIO, 다리오돔 등)    [nav_msgs/Odometry]
#   - 포인트클라우드 (도착시각·점개수만)       [sensor_msgs/PointCloud2]
#
#   각 토픽 -> 각각의 CSV. 전부 wall_t(time.time) 도착시각 기준.
#
# 실행 예:
#   source ~/fastlio_ws/install/setup.bash
#   python3 timesync_lio_logger.py \
#     --imu-topics /utlidar/imu /수정IMU토픽 \
#     --lowstate-topic /lowstate \
#     --odom-topics /Odometry /utlidar/robot_odom \
#     --cloud-topic /utlidar/cloud \
#     --outdir ./lio_log
#   (다른 터미널) LIO 실행 -> ros2 bag play ~/data/bags/ekf_test1  (배속 1.0)
#   재생 끝나면 Ctrl+C.
# =============================================================================
import argparse, os, time, csv, re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


def hs(msg):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


def safe(t):
    return re.sub(r'[^0-9a-zA-Z]+', '_', t).strip('_')


class Logger(Node):
    def __init__(self, args):
        super().__init__('ts_multi_logger')
        # BEST_EFFORT + 큰 버퍼: 어떤 publisher QoS 와도 호환되며 로거측 드롭 최소화
        qos = QoSProfile(depth=800, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.store = {}   # key(str) -> (header_list, rows_list)

        for t in args.imu_topics:
            k = f'imu__{safe(t)}'
            self.store[k] = (['wall_t', 'hstamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'], [])
            self.create_subscription(Imu, t, self._imu(k), qos)
        for t in args.odom_topics:
            k = f'odom__{safe(t)}'
            self.store[k] = (['wall_t', 'hstamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'], [])
            self.create_subscription(Odometry, t, self._odom(k), qos)
        if args.cloud_topic:
            k = f'cloud__{safe(args.cloud_topic)}'
            self.store[k] = (['wall_t', 'hstamp', 'npoints', 'row_step'], [])
            self.create_subscription(PointCloud2, args.cloud_topic, self._cloud(k), qos)
        if args.lowstate_topic:
            from unitree_go.msg import LowState   # ws source 되어 있어야 함
            k = f'lowstate__{safe(args.lowstate_topic)}'
            self.store[k] = (['wall_t', 'tick', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'], [])
            self.create_subscription(LowState, args.lowstate_topic, self._low(k), qos)

        self.get_logger().info('구독 토픽: ' + ', '.join(self.store.keys()))
        self.get_logger().info('LIO + bag play 시작하세요. 끝나면 Ctrl+C.')

    def _imu(self, k):
        def cb(m):
            a = m.linear_acceleration; g = m.angular_velocity
            self.store[k][1].append([time.time(), hs(m), a.x, a.y, a.z, g.x, g.y, g.z])
        return cb

    def _odom(self, k):
        def cb(m):
            p = m.pose.pose.position; q = m.pose.pose.orientation
            self.store[k][1].append([time.time(), hs(m), p.x, p.y, p.z,
                                     q.x, q.y, q.z, q.w])
        return cb

    def _cloud(self, k):
        def cb(m):
            self.store[k][1].append([time.time(), hs(m),
                                     m.width * m.height, m.row_step])
        return cb

    def _low(self, k):
        def cb(m):
            a = m.imu_state.accelerometer; g = m.imu_state.gyroscope
            self.store[k][1].append([time.time(), float(m.tick),
                                     a[0], a[1], a[2], g[0], g[1], g[2]])
        return cb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--imu-topics', nargs='*', default=[],
                    help='sensor_msgs/Imu 토픽들 (라이다IMU, 수정IMU 등)')
    ap.add_argument('--odom-topics', nargs='*', default=[],
                    help='nav_msgs/Odometry 토픽들 (LIO, 다리오돔 등)')
    ap.add_argument('--lowstate-topic', default='',
                    help='unitree_go/LowState (몸통 IMU)')
    ap.add_argument('--cloud-topic', default='',
                    help='sensor_msgs/PointCloud2 (도착·점개수만 기록)')
    ap.add_argument('--outdir', default='./lio_log')
    args = ap.parse_args()

    if not (args.imu_topics or args.odom_topics or args.lowstate_topic or args.cloud_topic):
        ap.error('토픽을 하나도 안 줬습니다. --imu-topics 등으로 지정하세요.')

    rclpy.init()
    node = Logger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        os.makedirs(args.outdir, exist_ok=True)
        for k, (header, rows) in node.store.items():
            p = os.path.join(args.outdir, k + '.csv')
            with open(p, 'w', newline='') as f:
                w = csv.writer(f); w.writerow(header); w.writerows(rows)
            flag = '  [!] 0건 - 토픽명/QoS 확인' if len(rows) == 0 else ''
            print(f'[저장] {p}  ({len(rows)} rows){flag}')
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
