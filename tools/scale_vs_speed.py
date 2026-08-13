#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""구간별 GPS/오도 변위비를 속도에 대해 본다 — k 의 속도 의존성 검사"""
import sqlite3, glob, os, json, math, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SMS = get_message('unitree_go/msg/SportModeState')
STR = get_message('std_msgs/msg/String')

def m_per_deg(lat):
    p = math.radians(lat)
    return (111132.92 - 559.82*math.cos(2*p) + 1.175*math.cos(4*p),
            111412.84*math.cos(p) - 93.5*math.cos(3*p))

def load(d):
    con = sqlite3.connect(glob.glob(os.path.join(d, "*.db3"))[0]); cur = con.cursor()
    def grab(t):
        r = cur.execute("SELECT id FROM topics WHERE name=?", (t,)).fetchone()
        return cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                           (r[0],)).fetchall() if r else []
    O = np.array([(ts/1e9,)+ (m.position[0], m.position[1])
                  for ts, x in grab('/sportmodestate')
                  for m in [deserialize_message(x, SMS)]])
    G = []
    for ts, x in grab('/gnss'):
        try: j = json.loads(deserialize_message(x, STR).data)
        except Exception: continue
        if j.get('fixed') == 1: G.append((ts/1e9, j['latitude'], j['longitude']))
    con.close()
    return O, np.array(G)

WIN = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
for d in sys.argv[1:2] or [os.path.expanduser("~/data/bags/outdoor/0812/go2_loop1_0812_1449")]:
    O, G = load(d)
    if len(G) < 20: print(f"{os.path.basename(d)}: GPS 부족"); continue
    ml, mo = m_per_deg(G[0,1])
    P = np.column_stack([(G[:,2]-G[0,2])*mo, (G[:,1]-G[0,1])*ml])
    ox = np.interp(G[:,0], O[:,0], O[:,1]); oy = np.interp(G[:,0], O[:,0], O[:,2])
    Q = np.column_stack([ox, oy])
    t = G[:,0]-G[0,0]
    rows = []
    i = 0
    while True:
        j = np.searchsorted(t, t[i]+WIN)
        if j >= len(t): break
        dg = np.linalg.norm(P[j]-P[i]); do = np.linalg.norm(Q[j]-Q[i])
        if do > 1.0:
            rows.append((do/(t[j]-t[i]), dg/do))
        i = j
    if not rows: print("구간 없음"); continue
    R = np.array(rows)
    print(f"\n{os.path.basename(d)}  창 {WIN:.0f}s, 구간 {len(R)}개")
    print(f"  {'속도대(m/s)':<14}{'구간수':>6}{'비율(GPS/오도)':>16}{'표준편차':>10}")
    for lo, hi in [(0,0.5),(0.5,0.9),(0.9,1.3),(1.3,2.0),(2.0,5.0)]:
        m = (R[:,0]>=lo)&(R[:,0]<hi)
        if m.sum() >= 3:
            print(f"  {lo:.1f}~{hi:.1f}{'':<7}{m.sum():>6}{R[m,1].mean():>16.4f}{R[m,1].std():>10.4f}")
