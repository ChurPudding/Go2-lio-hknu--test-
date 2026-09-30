#!/usr/bin/env python3
# PL 자세로 회전한 body 속도(R̂ᵀ·Δp/Δt) vs 다리 body 속도 → 프레임 정렬 + 스케일 k
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

pl, leg = read(sys.argv[1])
DT = 0.3                                   # 속도 미분 창 [s]
rows = []
for i in range(len(pl)):
    j = np.searchsorted(pl[:, 0], pl[i, 0] + DT)
    if j >= len(pl): break
    dt = pl[j, 0] - pl[i, 0]
    if dt <= 0: continue
    v_b = quat_to_R(*pl[i, 4:8]).T @ ((pl[j, 1:4] - pl[i, 1:4]) / dt)   # R̂ᵀ v̂  (코드의 Rtv)
    k = np.searchsorted(leg[:, 0], pl[i, 0] + dt/2)
    if k >= len(leg): break
    rows.append(np.concatenate([v_b, leg[k, 1:4]]))
A = np.array(rows)
B = A[np.linalg.norm(A[:, 3:5], axis=1) > 0.2]      # 다리 xy 속력 0.2 m/s 이상 = 걷는 중
print(f'걷는 샘플 {len(B)} / 전체 {len(A)}')
print('평균 PL body v (x,y,z):', np.round(B[:, 0:3].mean(0), 3))
print('평균 leg     v (x,y,z):', np.round(B[:, 3:6].mean(0), 3))
ratio = np.linalg.norm(B[:, 0:2], axis=1) / np.linalg.norm(B[:, 3:5], axis=1)
print(f'속력비 |PL|/|leg| 중앙값 = {np.median(ratio):.3f}   <- leg_scale 후보')
print(f'x축 상관 {np.corrcoef(B[:,0], B[:,3])[0,1]:.3f} | y축 상관 {np.corrcoef(B[:,1], B[:,4])[0,1]:.3f}')
