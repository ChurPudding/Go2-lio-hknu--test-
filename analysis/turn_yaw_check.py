#!/usr/bin/env python3
# 회전 구간별 yaw 증분: L1자이로 적분 / 몸통자이로 적분 / Point-LIO / 다리odom  (+ 자이로 바이어스, L1-몸통 시간차)
import sys, os, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
sys.path.insert(0, os.path.expanduser('~/fastlio_ws/tools')); from go2_calib import R_LB

def quat_to_R(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                     [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])
def yaw_of(R): return np.arctan2(R[1, 0], R[0, 0])
def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
def opened(bag):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', '')); return r

raw, plbag = sys.argv[1], sys.argv[2]
l1, body, leg, pl = [], [], [], []
r = opened(raw)
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/utlidar/imu':
        m = deserialize_message(d, Imu); w = m.angular_velocity; l1.append((stamp(m), w.x, w.y, w.z))
    elif t == '/utlidar/robot_odom':
        m = deserialize_message(d, Odometry); w = m.twist.twist.angular; q = m.pose.pose.orientation
        body.append((stamp(m), w.x, w.y, w.z)); leg.append((stamp(m), yaw_of(quat_to_R(q.w, q.x, q.y, q.z))))
r = opened(plbag)
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/aft_mapped_to_init':
        m = deserialize_message(d, Odometry); q = m.pose.pose.orientation
        pl.append((stamp(m), yaw_of(quat_to_R(q.w, q.x, q.y, q.z) @ R_LB)))      # R_wb = R_wL1 · R_LB
l1, body, leg, pl = (np.array(sorted(a)) for a in (l1, body, leg, pl))
T0 = body[0, 0]
wz_l1 = (l1[:, 1:4] @ R_LB)[:, 2]                                                 # ω_body = R_LBᵀ ω_L1, z = yaw rate
wz_body = body[:, 3]
b_l1 = wz_l1[l1[:, 0] < T0 + 60].mean(); b_body = wz_body[body[:, 0] < T0 + 60].mean()
integ = lambda t, w: np.concatenate([[0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(t))])
Y = {'L1자이로': (l1[:, 0], integ(l1[:, 0], wz_l1 - b_l1)), '몸통자이로': (body[:, 0], integ(body[:, 0], wz_body - b_body)),
     'Point-LIO': (pl[:, 0], np.unwrap(pl[:, 1])), '다리odom': (leg[:, 0], np.unwrap(leg[:, 1]))}
g = np.arange(max(l1[0, 0], body[0, 0]), min(l1[-1, 0], body[-1, 0]), 0.005)
a = np.interp(g, l1[:, 0], wz_l1) - b_l1; b = np.interp(g, body[:, 0], wz_body) - b_body
lags = np.arange(-100, 101)
cc = [np.dot(a[max(0, k):len(a) + min(0, k)], b[max(0, -k):len(b) + min(0, -k)]) for k in lags]
print(f'자이로 바이어스(정지 60 s): L1 {np.degrees(b_l1):+.3f}°/s   몸통 {np.degrees(b_body):+.3f}°/s')
print(f'L1 자이로가 몸통 자이로보다 {lags[int(np.argmax(cc)) ] * 5:+d} ms 늦음(+)/빠름(−)')
w_s = np.convolve(np.abs(wz_body - b_body), np.ones(45) / 45, mode='same'); on = w_s > 0.3
segs, i = [], 0
while i < len(on):
    if on[i]:
        j = i
        while j < len(on) and on[j]: j += 1
        segs.append([body[i, 0], body[j - 1, 0]]); i = j
    else: i += 1
merged = []
for s in segs:
    if merged and s[0] - merged[-1][1] < 1.0: merged[-1][1] = s[1]
    else: merged.append(s)
merged = [s for s in merged if s[1] - s[0] >= 1.0]
print(f'\n회전 구간 {len(merged)}개  (Δyaw [deg], 구간 ±0.5 s)')
print(f'{"#":>2} {"시작[s]":>7} {"길이[s]":>6} | ' + ' '.join(f'{k:>10s}' for k in Y) + ' |  PL−다리')
tot = {k: 0.0 for k in Y}
for n, (s0, s1) in enumerate(merged, 1):
    d = {k: np.degrees(np.interp(s1 + 0.5, t, y) - np.interp(s0 - 0.5, t, y)) for k, (t, y) in Y.items()}
    for k in Y: tot[k] += abs(d[k])
    print(f'{n:>2} {s0 - T0:7.1f} {s1 - s0:6.1f} | ' + ' '.join(f'{d[k]:+10.1f}' for k in Y) + f' | {d["Point-LIO"] - d["다리odom"]:+7.1f}')
print(f'{"Σ|Δ|":>17} | ' + ' '.join(f'{tot[k]:10.1f}' for k in Y))
print('비율(몸통자이로 기준): ' + '  '.join(f'{k} {tot[k] / tot["몸통자이로"]:.3f}' for k in Y))
# ---- 직진 구간(회전 사이)에서의 PL−다리 yaw 변화 = LiDAR 슬라이드 ----
bounds = [(T0 + 65, merged[0][0] - 0.5)] + [(merged[i][1] + 0.5, merged[i + 1][0] - 0.5) for i in range(len(merged) - 1)] + [(merged[-1][1] + 0.5, pl[-1, 0])]
gaps = []
for g0, g1 in bounds:
    if g1 - g0 < 1.0: continue
    dpl = np.degrees(np.interp(g1, *Y['Point-LIO']) - np.interp(g0, *Y['Point-LIO']))
    dlg = np.degrees(np.interp(g1, *Y['다리odom']) - np.interp(g0, *Y['다리odom']))
    gaps.append((dpl - dlg, g0 - T0, g1 - g0, dpl, dlg))
turn_sum = sum(np.degrees(np.interp(s1 + 0.5, *Y['Point-LIO']) - np.interp(s0 - 0.5, *Y['Point-LIO']) - np.interp(s1 + 0.5, *Y['다리odom']) + np.interp(s0 - 0.5, *Y['다리odom'])) for s0, s1 in merged)
print(f'\n직진 구간 PL−다리 변화 (큰 순서 8개)   회전 합계 {turn_sum:+.1f}°   직진 합계 {sum(g[0] for g in gaps):+.1f}°')
for e, s, L, dpl, dlg in sorted(gaps, key=lambda x: -abs(x[0]))[:8]:
    print(f'  시작 {s:6.1f}s  길이 {L:5.1f}s | PL {dpl:+7.1f}  다리 {dlg:+7.1f} | 슬라이드 {e:+6.1f}°')
