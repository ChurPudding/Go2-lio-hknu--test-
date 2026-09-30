#!/usr/bin/env python3
# z(t)·pitch(t) 시계열 비교 (정지 구간 음영은 첫 bag의 다리 속도 기준)
import sys, os, numpy as np, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
sys.path.insert(0, os.path.expanduser('~/fastlio_ws/tools')); from go2_calib import R_LB
def quat_to_R(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])
def read(bag):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    pl, leg = [], []
    while r.has_next():
        t, d, _ = r.read_next()
        if t == '/aft_mapped_to_init':
            m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation
            Rb = quat_to_R(q.w, q.x, q.y, q.z) @ R_LB                       # 몸통 자세
            pl.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, p.z, np.degrees(-np.arcsin(np.clip(Rb[2, 0], -1, 1)))))
        elif t == '/utlidar/robot_odom':
            m = deserialize_message(d, Odometry); v = m.twist.twist.linear
            leg.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, np.hypot(v.x, v.y)))
    return np.array(sorted(pl)), np.array(sorted(leg))
bags = sys.argv[1:]; fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
_, leg0 = read(bags[0]); T0 = leg0[0, 0]
still = np.convolve(leg0[:, 1] < 0.05, np.ones(45)/45, mode='same') > 0.5
for a in ax: a.fill_between(leg0[:, 0]-T0, 0, 1, where=still, transform=a.get_xaxis_transform(), color='0.85')
for b in bags:
    pl, _ = read(b); t = pl[:, 0] - T0; name = os.path.basename(b)
    ax[0].plot(t, pl[:, 1], lw=1, label=f'{name}  min {pl[:,1].min():+.2f}  end {pl[-1,1]:+.2f} m')
    ax[1].plot(t, pl[:, 2], lw=1, label=name)
    i = np.argmin(pl[:, 1]); print(f'{name:22s} z 최소 {pl[i,1]:+.2f} m @ {t[i]:6.1f} s | 정지#2(249~315) 평균 z {pl[(t>249)&(t<315),1].mean():+.2f} | 끝 {pl[-1,1]:+.2f}')
ax[0].axhline(0, color='k', lw=.5); ax[0].set_ylabel('z [m]'); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
ax[1].set_ylabel('body pitch [deg]'); ax[1].set_xlabel('time [s]'); ax[1].grid(alpha=.3)
fig.savefig(os.path.expanduser('~/data/bags/z_compare.png'), dpi=130); print('그림 저장: ~/data/bags/z_compare.png')
