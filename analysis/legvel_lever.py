#!/usr/bin/env python3
# 회전 구간에서 레버암 항 효과: 다리(base 원점) vs PL(L1 원점) 속도 잔차, 보정 없음 / +ω×r / -ω×r
import sys, os, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
sys.path.insert(0, os.path.expanduser('~/fastlio_ws/tools')); from go2_calib import R_LB, LEVER
K, DT = 0.95, 0.3
def quat_to_R(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
pl, leg = [], []
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/aft_mapped_to_init':
        m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation
        pl.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, p.x, p.y, p.z, q.w, q.x, q.y, q.z))
    elif t == '/utlidar/robot_odom':
        m = deserialize_message(d, Odometry); v = m.twist.twist.linear; w = m.twist.twist.angular
        leg.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, v.x, v.y, v.z, w.x, w.y, w.z))
pl, leg = np.array(sorted(pl)), np.array(sorted(leg)); rL = R_LB @ LEVER
res = {'보정 없음': [], '+ω×r': [], '-ω×r': []}; wz = []
for i in range(len(pl)):
    j = np.searchsorted(pl[:, 0], pl[i, 0] + DT)
    if j >= len(pl): break
    dt = pl[j, 0] - pl[i, 0]
    if dt <= 0: continue
    k = np.searchsorted(leg[:, 0], pl[i, 0] + dt/2)
    if k >= len(leg): break
    if np.hypot(leg[k, 1], leg[k, 2]) < 0.2: continue
    vL1 = quat_to_R(*pl[i, 4:8]).T @ ((pl[j, 1:4] - pl[i, 1:4]) / dt)   # PL: L1 원점 속도 (L1 프레임)
    z = R_LB @ (K * leg[k, 1:4]); wL = R_LB @ leg[k, 4:7]                # 다리: base 원점 속도 / 각속도 (L1 프레임)
    res['보정 없음'].append((z - vL1)[:2]); res['+ω×r'].append((z + np.cross(wL, rL) - vL1)[:2]); res['-ω×r'].append((z - np.cross(wL, rL) - vL1)[:2])
    wz.append(abs(leg[k, 6]))
wz = np.array(wz); masks = [('전체', wz >= 0), ('회전 중 |ωz|>0.3', wz > 0.3), ('직진 |ωz|<0.1', wz < 0.1)]
print(f'r_L (L1 프레임) = {np.round(rL, 3)}')
for name, mask in masks:
    rms = {k: np.sqrt((np.array(v)[mask] ** 2).sum(1).mean()) for k, v in res.items()}
    print(f'{name:16s} n={mask.sum():5d} | xy 잔차 RMS  ' + '   '.join(f'{k} {v:.3f}' for k, v in rms.items()) + ' m/s')
