#!/usr/bin/env python3
# 정지 구간 진단: 창별 다리 속도 / zupt_active 비율 / PL yaw 변화율
import sys, os, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
sys.path.insert(0, os.path.expanduser('~/fastlio_ws/tools')); from go2_calib import R_LB
def quat_to_R(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])
stamp = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
pl, leg, zu, off = [], [], [], []
while r.has_next():
    t, d, tn = r.read_next()
    if t == '/aft_mapped_to_init':
        m = deserialize_message(d, Odometry); q = m.pose.pose.orientation; R = quat_to_R(q.w, q.x, q.y, q.z) @ R_LB
        pl.append((stamp(m), np.arctan2(R[1, 0], R[0, 0])))
    elif t == '/utlidar/robot_odom':
        m = deserialize_message(d, Odometry); v = m.twist.twist.linear; w = m.twist.twist.angular
        leg.append((stamp(m), np.hypot(v.x, v.y), v.z, w.z)); off.append(tn * 1e-9 - stamp(m))
    elif t == '/zupt_active':
        zu.append((tn * 1e-9, float(deserialize_message(d, Bool).data)))
pl, leg, zu = np.array(sorted(pl)), np.array(sorted(leg)), np.array(sorted(zu))
zu[:, 0] -= np.median(off)                     # 수신시각 -> bag 시각
T0 = leg[0, 0]; yaw = np.unwrap(pl[:, 1])
print(f'{"구간":10s} {"다리|vxy| 평균/최대":>19s} {"vz평균":>7s} {"|wz|평균":>8s} {"zupt true":>10s} {"PL yaw율":>10s}')
for a, b, name in [(0, 25, '엎드림'), (25, 65, '서서정지'), (65, 120, '걷기시작'), (249, 315, '정지#2')]:
    L = leg[(leg[:, 0] >= T0 + a) & (leg[:, 0] < T0 + b)]; Z = zu[(zu[:, 0] >= T0 + a) & (zu[:, 0] < T0 + b)]
    y0, y1 = np.interp(T0 + a, pl[:, 0], yaw), np.interp(T0 + b, pl[:, 0], yaw)
    print(f'{name:10s} {L[:, 1].mean():9.3f}/{L[:, 1].max():<9.3f} {L[:, 2].mean():7.3f} {np.abs(L[:, 3]).mean():8.3f} '
          f'{Z[:, 1].mean() if len(Z) else float("nan"):10.2f} {np.degrees(y1 - y0) / (b - a):+9.2f}°/s')
