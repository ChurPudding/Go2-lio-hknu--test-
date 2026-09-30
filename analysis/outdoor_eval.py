#!/usr/bin/env python3
# outdoor_eval.py — 야외 bag: Point-LIO(OFF/ON)·다리 궤적을 GPS 기준으로 비교
# 사용: python3 ~/data/bags/outdoor_eval.py <원본 bag> <출력 bag> [출력 bag ...]
import sys, os, json, math
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

K = 1.23

def reader(path, topics):
    with open(os.path.join(path, 'metadata.yaml')) as f:
        m = yaml.safe_load(f)['rosbag2_bagfile_information']
    r = rosbag2_py.SequentialCompressionReader() if m.get('compression_mode') else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=m.get('storage_identifier', 'sqlite3')),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r

def hdr(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9

def plen(tr, dt):
    L, last = 0.0, None
    for row in tr:
        if last is None or row[0] - last[0] >= dt:
            if last is not None:
                L += math.hypot(row[1] - last[1], row[2] - last[2])
            last = row
    return L

def ends(tr, win=5.0):
    return (tr[tr[:, 0] <= tr[0, 0] + win, 1:3].mean(0),
            tr[tr[:, 0] >= tr[-1, 0] - win, 1:3].mean(0))

def kabsch2(A, B):
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca

src = sys.argv[1].rstrip('/')
r, offs = reader(src, ['/utlidar/cloud']), []
while r.has_next() and len(offs) < 300:
    _, data, t = r.read_next()
    offs.append(t * 1e-9 - hdr(deserialize_message(data, PointCloud2).header))
off = float(np.median(offs))

gps, leg = [], []
r = reader(src, ['/gnss', '/utlidar/robot_odom'])
while r.has_next():
    topic, data, t = r.read_next()
    t *= 1e-9
    if topic == '/gnss':
        try:
            g = json.loads(deserialize_message(data, String).data)
            if str(g.get('fixed', 0)).lower() in ('1', '1.0', 'true') and 0 < float(g.get('hdop', 99)) < 90:
                gps.append((t, float(g['latitude']), float(g['longitude'])))
        except Exception:
            pass
    else:
        p = deserialize_message(data, Odometry).pose.pose.position
        leg.append((t, p.x, p.y, p.z))
if len(gps) < 10:
    sys.exit('GPS 고정 샘플이 10개 미만 — 이 bag은 GPS 기준 평가 불가')
gps, leg = np.array(gps), np.array(leg)
G = np.c_[gps[:, 0], (gps[:, 2] - gps[0, 2]) * 111320 * math.cos(math.radians(gps[0, 1])),
          (gps[:, 1] - gps[0, 1]) * 111320]
leg[:, 1:3] = leg[0, 1:3] + K * (leg[:, 1:3] - leg[0, 1:3])
g0, g1 = ends(G)
dG = g1 - g0
curves = []

def evaluate(name, tr, show_z):
    m = (G[:, 0] >= tr[0, 0]) & (G[:, 0] <= tr[-1, 0])
    if m.sum() < 10:
        print(f'{name:24s} GPS와 겹치는 시간이 없음 (궤적 {tr[0, 0]:.1f}~{tr[-1, 0]:.1f}, GPS {G[0, 0]:.1f}~{G[-1, 0]:.1f})')
        return
    A = np.c_[np.interp(G[m, 0], tr[:, 0], tr[:, 1]), np.interp(G[m, 0], tr[:, 0], tr[:, 2])]
    R, t = kabsch2(A, G[m, 1:3])
    res = np.linalg.norm(A @ R.T + t - G[m, 1:3], axis=1)
    a, b = ends(tr)
    e_end = np.linalg.norm(R @ (b - a) - dG)
    z = f'{tr[-1, 3]:+.2f} / {tr[:, 3].min():+.2f}' if show_z else '-'
    print(f'{name:24s} {m.sum():5d} {math.sqrt((res ** 2).mean()):8.2f} {res.max():7.2f} {e_end:8.2f} '
          f'{np.linalg.norm(b - a):8.1f} {plen(tr, 0.5):7.0f}   {z}')
    curves.append((name, tr[:, 1:3] @ R.T + t))

print(f'원본 {os.path.basename(src)} · GPS 고정 {len(G)}점 · 시계 오프셋(수신−헤더) {off:+.2f} s\n')
print(f'{"":24s} {"매칭":>5s} {"정렬RMS":>8s} {"최대":>7s} {"끝점오차":>8s} {"시작↔끝":>8s} {"경로":>7s}   z 끝/최저')
print(f'{"GPS":24s} {len(G):5d} {"-":>8s} {"-":>7s} {"-":>8s} {np.linalg.norm(dG):8.1f} {plen(G, 2.0):7.0f}')
evaluate(f'leg×{K}', leg, False)
for p in sys.argv[2:]:
    rr, tr = reader(p.rstrip('/'), ['/aft_mapped_to_init']), []
    while rr.has_next():
        _, data, _ = rr.read_next()
        o = deserialize_message(data, Odometry)
        q = o.pose.pose.position
        tr.append((hdr(o.header) + off, q.x, q.y, q.z))
    evaluate(os.path.basename(p.rstrip('/')), np.array(tr), True)

plt.figure(figsize=(7, 7))
plt.plot(G[:, 1], G[:, 2], '.', color='0.6', ms=4, label='GPS')
for name, xy in curves:
    plt.plot(xy[:, 0], xy[:, 1], lw=1.2, label=name)
plt.plot(G[0, 1], G[0, 2], 'k^', ms=9)
plt.axis('equal'); plt.grid(alpha=0.3); plt.legend(); plt.xlabel('E [m]'); plt.ylabel('N [m]')
plt.title(os.path.basename(src))
out = os.path.join(os.path.dirname(src), f'outdoor_{os.path.basename(src)[-4:]}.png')
plt.savefig(out, dpi=130, bbox_inches='tight')
print(f'\n그림 저장: {out}')
