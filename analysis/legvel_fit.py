#!/usr/bin/env python3
# OFF bag에서 base_link(다리) → Point-LIO body(IMU) 회전 R_ib 와 스케일 k 추정 (Kabsch)
import sys, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    pl, leg = [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/aft_mapped_to_init':
            m = deserialize_message(data, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation
            pl.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, p.x, p.y, p.z, q.w, q.x, q.y, q.z))
        elif topic == '/utlidar/robot_odom':
            m = deserialize_message(data, Odometry); v = m.twist.twist.linear
            leg.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, v.x, v.y, v.z))
    return np.array(sorted(pl)), np.array(sorted(leg))

def quat_to_R(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])

def rpy(R):  # ZYX: R = Rz(yaw) Ry(pitch) Rx(roll)
    return np.degrees([np.arctan2(R[2,1], R[2,2]), -np.arcsin(R[2,0]), np.arctan2(R[1,0], R[0,0])])

pl, leg = read(sys.argv[1]); DT = 0.3; P, L = [], []
for i in range(len(pl)):
    j = np.searchsorted(pl[:, 0], pl[i, 0] + DT)
    if j >= len(pl): break
    dt = pl[j, 0] - pl[i, 0]
    if dt <= 0: continue
    k = np.searchsorted(leg[:, 0], pl[i, 0] + dt/2)
    if k >= len(leg): break
    if np.hypot(leg[k, 1], leg[k, 2]) < 0.2: continue                    # 걷는 구간만
    P.append(quat_to_R(*pl[i, 4:8]).T @ ((pl[j, 1:4] - pl[i, 1:4]) / dt)); L.append(leg[k, 1:4])
P, L = np.array(P), np.array(L)
U, S, Vt = np.linalg.svd(L.T @ P)                                        # P ≈ R L
D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
R = Vt.T @ D @ U.T
RL = L @ R.T
k = (P * RL).sum() / (RL * RL).sum()                                     # 최소제곱 스케일
res = P - k * RL
print(f'샘플 {len(P)}   특이값/N {np.round(S/len(P), 3)}  (작은 축 = 약하게 결정)')
print('R_ib (base→IMU) =\n', np.round(R, 4))
print('rpy [deg] (roll, pitch, yaw) =', np.round(rpy(R), 1))
print(f'스케일 k = {k:.3f}')
print(f'x축 상관: 회전 전 {np.corrcoef(P[:,0], L[:,0])[0,1]:.3f} → 회전 후 {np.corrcoef(P[:,0], RL[:,0])[0,1]:.3f}')
print('회전+스케일 후 잔차 평균 (x,y,z):', np.round(res.mean(0), 3), ' 표준편차:', np.round(res.std(0), 3))
import os; sys.path.insert(0, os.path.expanduser('~/fastlio_ws/tools')); from go2_calib import R_LB
RL2 = L @ R_LB.T
k_xy = (P[:, :2] * RL2[:, :2]).sum() / (RL2[:, :2] ** 2).sum()
res2 = P[:, :2] - k_xy * RL2[:, :2]
print(f'[R_LB 고정, xy] k_xy = {k_xy:.3f}   잔차 평균 {np.round(res2.mean(0), 3)}  표준편차 {np.round(res2.std(0), 3)}')
print(f'전진축 각도차 (fit vs R_LB) = {np.degrees(np.arccos(np.dot(R[:, 0], R_LB[:, 0]))):.1f} deg')
