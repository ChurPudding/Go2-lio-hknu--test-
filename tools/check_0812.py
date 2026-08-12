#!/usr/bin/env python3
import sqlite3, glob, os, json, math, sys
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

BASE = os.path.expanduser("~/data/bags/outdoor/0812")
SMS  = get_message('unitree_go/msg/SportModeState')

def read(d):
    db = glob.glob(os.path.join(d, "*.db3"))
    if not db: return None, None
    con = sqlite3.connect(db[0]); cur = con.cursor()
    def grab(topic):
        r = cur.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
        if not r: return []
        return cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                           (r[0],)).fetchall()
    sms, gnss = grab('/sportmodestate'), grab('/gnss')
    con.close()
    return sms, gnss

for d in sorted(glob.glob(os.path.join(BASE, "go2_*")) + glob.glob(os.path.join(BASE, "gps_*"))):
    name = os.path.basename(d)
    sms, gnss = read(d)
    if sms is None: continue
    print(f"\n=== {name} ===")

    # 1) 위치 점프 = 전원 리셋 흔적
    if sms:
        t0 = sms[0][0]; prev = None; jumps = []
        pts = []
        for ts, data in sms:
            m = deserialize_message(data, SMS)
            p = (m.position[0], m.position[1], m.position[2])
            yaw = m.imu_state.rpy[2]
            pts.append((ts, p, yaw))
            if prev:
                dd = math.dist(p[:2], prev[:2])
                if dd > 0.5: jumps.append(((ts-t0)/1e9, dd))
            prev = p
        dur = (sms[-1][0]-t0)/1e9
        print(f"  sportmodestate: {len(sms)}개, {dur:.1f}s")
        if jumps:
            print(f"  !! 위치 점프 {len(jumps)}회 (전원 리셋 의심)")
            for t, dd in jumps[:3]: print(f"     t={t:.1f}s  {dd:.2f}m")
        else:
            print("  위치 연속 — 리셋 없음")

        # 2) 폐루프 오차 (오도메트리 기준)
        s, e = pts[0], pts[-1]
        dx, dy = e[1][0]-s[1][0], e[1][1]-s[1][1]
        dz = e[1][2]-s[1][2]
        dyaw = math.degrees(e[2]-s[2])
        dyaw = (dyaw+180) % 360 - 180
        # 총 이동거리
        L = sum(math.dist(pts[i][1][:2], pts[i-1][1][:2]) for i in range(1,len(pts)))
        err = math.hypot(dx, dy)
        print(f"  경로장 {L:.1f}m")
        print(f"  시종점차 dx={dx:+.2f} dy={dy:+.2f} dz={dz:+.2f} m, |수평|={err:.2f}m")
        if L > 1: print(f"  드리프트 {err/L*100:.2f}%")
        print(f"  yaw 차 {dyaw:+.2f}deg")

    # 3) GPS 상태
    if gnss:
        fixed = {}; inuse = []
        for ts, data in gnss:
            try:
                j = json.loads(deserialize_message(data, get_message('std_msgs/msg/String')).data)
            except Exception: continue
            fixed[j.get('fixed')] = fixed.get(j.get('fixed'),0)+1
            inuse.append(j.get('satellite_inuse',0))
        print(f"  gnss: {len(gnss)}개, fixed분포={fixed}, inuse평균={sum(inuse)/max(len(inuse),1):.1f}")
    else:
        print("  gnss: 0개  <-- 없음")
