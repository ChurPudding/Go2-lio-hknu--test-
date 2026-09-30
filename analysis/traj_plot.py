#!/usr/bin/env python3
# 여러 bag의 PL 궤적 + 다리 궤적(첫 bag)을 xy에 겹쳐 그림. 다리는 첫 8 m 직진 방향으로 PL에 정렬(회전+평행이동).
import sys, os, numpy as np, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def read(bag):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    pl, leg = [], []
    while r.has_next():
        t, d, _ = r.read_next()
        if t not in ('/aft_mapped_to_init', '/utlidar/robot_odom'): continue
        m = deserialize_message(d, Odometry); p = m.pose.pose.position
        (pl if t == '/aft_mapped_to_init' else leg).append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, p.x, p.y))
    return np.array(sorted(pl)), np.array(sorted(leg))

def first_dir(P, L=8.0):
    d = np.hypot(*(P[:, 1:3] - P[0, 1:3]).T); i = int(np.argmax(d > L)); v = P[i, 1:3] - P[0, 1:3]
    return np.arctan2(v[1], v[0])

bags = sys.argv[1:]; fig, ax = plt.subplots(figsize=(9, 9))
pl0, leg = read(bags[0])
th = first_dir(pl0) - first_dir(leg); R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
legA = (leg[:, 1:3] - leg[0, 1:3]) @ R.T + pl0[0, 1:3]
e = np.hypot(*(legA[-1] - legA[0])); ax.plot(legA[:, 0], legA[:, 1], color='0.6', lw=2.5, label=f'leg odom (aligned)  end err {e:.1f} m')
for b in bags:
    pl = pl0 if b == bags[0] else read(b)[0]
    e = np.hypot(*(pl[-1, 1:3] - pl[0, 1:3])); ax.plot(pl[:, 1], pl[:, 2], lw=1.2, label=f'PL {os.path.basename(b)}  end err {e:.1f} m')
    ax.plot(pl[-1, 1], pl[-1, 2], 'x', ms=9, color=ax.lines[-1].get_color())
ax.plot(pl0[0, 1], pl0[0, 2], 'ko', ms=8, label='start'); ax.set_aspect('equal'); ax.grid(alpha=.3)
ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.legend(loc='best'); ax.set_title('xy trajectories (x = end)')
fig.savefig(os.path.expanduser('~/data/bags/traj_compare.png'), dpi=130); print('그림 저장: ~/data/bags/traj_compare.png')
