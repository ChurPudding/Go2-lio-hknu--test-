#!/usr/bin/env python3
# outdoor_scan.py — 지정 날짜(KST) bag: 레그 A/B 가능 여부 · 권장 --start-offset · 루프 여부 · GPS 품질
# 사용: python3 ~/data/bags/outdoor_scan.py [YYYY-MM-DD] [루트]   (기본 2026-09-25, ~/data/bags)
import sys, os, json, math, datetime as dt
import yaml

DATE = sys.argv[1] if len(sys.argv) > 1 else '2026-09-25'
ROOT = os.path.expanduser(sys.argv[2] if len(sys.argv) > 2 else '~/data/bags')
K = 1.23
NEED = ['/utlidar/cloud', '/utlidar/imu', '/lowstate', '/utlidar/robot_odom', '/gnss']
KST = dt.timezone(dt.timedelta(hours=9))

def load_meta(d):
    with open(os.path.join(d, 'metadata.yaml')) as f:
        m = yaml.safe_load(f)['rosbag2_bagfile_information']
    t0 = dt.datetime.fromtimestamp(m['starting_time']['nanoseconds_since_epoch'] / 1e9, KST)
    cnt = {x['topic_metadata']['name']: x['message_count'] for x in m['topics_with_message_count']}
    return m, t0, m['duration']['nanoseconds'] / 1e9, cnt

bags = []
for d, _, files in os.walk(ROOT):
    if 'metadata.yaml' in files:
        try:
            bags.append((d,) + load_meta(d))
        except Exception as ex:
            print(f'[읽기 실패] {d}: {ex}')
    elif any(f.endswith(('.db3', '.mcap')) for f in files):
        mt = max(os.path.getmtime(os.path.join(d, f)) for f in files if f.endswith(('.db3', '.mcap')))
        if f'{dt.datetime.fromtimestamp(mt, KST):%Y-%m-%d}' == DATE:
            print(f'[metadata 없음] {os.path.relpath(d, ROOT)} — 녹화가 비정상 종료된 bag')
bags.sort(key=lambda b: b[2])
day = [b for b in bags if f'{b[2]:%Y-%m-%d}' == DATE]
print(f'{ROOT} 아래 bag {len(bags)}개 중 {DATE} 녹화 {len(day)}개\n')
if not day:
    print('가장 최근 bag 8개:')
    for d, m, t0, dur, cnt in bags[-8:]:
        print(f'  {t0:%m-%d %H:%M}  {dur:6.0f} s  {os.path.relpath(d, ROOT)}')
    sys.exit()

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def read(d, m, topics):
    r = rosbag2_py.SequentialCompressionReader() if m.get('compression_mode') else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=d, storage_id=m.get('storage_identifier', 'sqlite3')),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    out = {t: [] for t in topics}
    while r.has_next():
        topic, data, stamp = r.read_next()
        out[topic].append((stamp * 1e-9, data))
    return out

def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else float('nan')

for i, (d, m, t0, dur, cnt) in enumerate(day, 1):
    print(f'[{i}] {os.path.relpath(d, ROOT)}   {t0:%H:%M} 시작 · {dur:.0f} s')
    print('    ' + ' | '.join(f"{t.split('/')[-1]} {cnt.get(t, 0)}" for t in NEED))
    miss = [t for t in NEED[:4] if not cnt.get(t)]
    if miss:
        print(f'    → 레그 A/B 불가 ({", ".join(miss)} 없음)\n')
        continue
    raw = read(d, m, ['/utlidar/robot_odom'] + (['/gnss'] if cnt.get('/gnss') else []))
    tb0 = m['starting_time']['nanoseconds_since_epoch'] * 1e-9
    odo = [(t - tb0, deserialize_message(x, Odometry)) for t, x in raw['/utlidar/robot_odom']]
    bins = {}
    for t, o in odo:
        bins.setdefault(int(t / 0.5), []).append(math.hypot(o.twist.twist.linear.x, o.twist.twist.linear.y))
    fast = {k for k, v in bins.items() if sum(v) / len(v) > 0.15}
    t_move = next((k * 0.5 for k in sorted(fast) if k + 1 in fast and k + 2 in fast), None)
    tm = t_move if t_move is not None else odo[-1][0]
    pre = [(t, o.pose.pose.position.z) for t, o in odo if t <= tm] or [(0.0, odo[0][1].pose.pose.position.z)]
    zlo, zhi = min(z for _, z in pre), max(z for _, z in pre)
    t_up = next((t for t, z in pre if z >= zlo + 0.8 * (zhi - zlo)), None) if zhi - zlo > 0.1 else None
    if t_move is None:
        off = '이동 구간 없음'
    elif t_up is not None:
        off = f'{max(0.0, min(t_up + 5, t_move - 5)):.0f}'
    else:
        off = '0' if t_move >= 8 else '정지 구간 부족 — 확인 필요'
    up = f'{t_up:.1f} s' if t_up is not None else '감지 안 됨'
    print(f'    이동 시작 {tm:.1f} s · 기상(odom z {zlo:.2f}→{zhi:.2f}) {up} → 권장 --start-offset {off}')
    zs = ' '.join(f'{min(pre, key=lambda q: abs(q[0] - tt))[1]:.2f}' for tt in range(0, int(min(tm, 60)) + 1, 5))
    print(f'    z (5 s 간격, 이동 전): {zs}')
    L, lt, lp = 0.0, None, None
    for t, o in odo:
        if lt is None or t - lt >= 0.5:
            p = o.pose.pose.position
            if lp:
                L += math.hypot(p.x - lp[0], p.y - lp[1])
            lt, lp = t, (p.x, p.y)
    def mxy(sel):
        return (sum(o.pose.pose.position.x for o in sel) / len(sel), sum(o.pose.pose.position.y for o in sel) / len(sel))
    s = mxy([o for t, o in odo if t <= max(tm, odo[0][0] + 1.0)])
    e = mxy([o for t, o in odo if t >= odo[-1][0] - 5])
    gap = K * math.hypot(e[0] - s[0], e[1] - s[1])
    kind = '루프' if gap < 3 + 0.05 * K * L else '편도'
    print(f'    다리×{K} 경로 {K * L:.0f} m · 시작↔끝 {gap:.1f} m → {kind}')
    if '/gnss' in raw:
        n = fx = 0
        sats, hd, pts, keys = [], [], [], None
        for t, x in raw['/gnss']:
            n += 1
            try:
                g = json.loads(deserialize_message(x, String).data)
            except Exception:
                continue
            if not isinstance(g, dict):
                continue
            keys = keys or list(g)
            fv = g.get('fixed', 0)
            if not (fv is True or str(fv).lower() in ('1', '1.0', 'true')):
                continue
            fx += 1
            if 'satellite_inuse' in g:
                sats.append(float(g['satellite_inuse']))
            if 'hdop' in g and float(g['hdop']) < 99:
                hd.append(float(g['hdop']))
            la = next((g[k] for k in g if k.lower() in ('lat', 'latitude')), None)
            lo = next((g[k] for k in g if k.lower() in ('lon', 'lng', 'longitude')), None)
            if la is not None and lo is not None:
                pts.append((float(la), float(lo)))
        line = f'    GPS 고정 {100 * fx / max(n, 1):.0f} % ({fx}/{n}) · 위성 중앙 {med(sats):.0f} · hdop 중앙 {med(hd):.1f}'
        if len(pts) >= 20:
            c = math.cos(math.radians(pts[0][0]))
            enu = [((q[1] - pts[0][1]) * 111320 * c, (q[0] - pts[0][0]) * 110540) for q in pts]
            a, b = enu[:10], enu[-10:]
            gs = math.hypot(sum(p[0] for p in b) / 10 - sum(p[0] for p in a) / 10,
                            sum(p[1] for p in b) / 10 - sum(p[1] for p in a) / 10)
            line += f' · 시작↔끝 {gs:.1f} m'
        elif keys:
            line += f' · (위경도 키를 못 찾음: {keys})'
        print(line)
    print()
