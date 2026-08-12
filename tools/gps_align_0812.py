#!/usr/bin/env python3
"""GPS ENU vs 다리 오도메트리 정렬 분석"""
import sqlite3, glob, os, json, math
import numpy as np
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

BASE = os.path.expanduser("~/data/bags/outdoor/0812")
SMS  = get_message('unitree_go/msg/SportModeState')
STR  = get_message('std_msgs/msg/String')

def load(d):
    db = glob.glob(os.path.join(d, "*.db3"))
    con = sqlite3.connect(db[0]); cur = con.cursor()
    def grab(t):
        r = cur.execute("SELECT id FROM topics WHERE name=?", (t,)).fetchone()
        return cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                           (r[0],)).fetchall() if r else []
    odo, gps = [], []
    for ts, data in grab('/sportmodestate'):
        m = deserialize_message(data, SMS)
        odo.append((ts/1e9, m.position[0], m.position[1], m.imu_state.rpy[2]))
    for ts, data in grab('/gnss'):
        try: j = json.loads(deserialize_message(data, STR).data)
        except Exception: continue
        if j.get('fixed') != 1: continue
        gps.append((ts/1e9, j['latitude'], j['longitude'], j.get('hdop',0)))
    con.close()
    return np.array(odo), np.array(gps)

def enu(gps):
    lat0, lon0 = gps[0,1], gps[0,2]
    E = (gps[:,2]-lon0) * 111320.0 * math.cos(math.radians(lat0))
    N = (gps[:,1]-lat0) * 110540.0
    return np.column_stack([E, N])

def fit(src, dst, with_scale):
    """src(오도메트리) -> dst(ENU). yaw + 평행이동 (+선택적 스케일)"""
    sc, dc = src.mean(0), dst.mean(0)
    A, B = src-sc, dst-dc
    H = A.T @ B
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1; R = Vt.T @ U.T
    s = (S.sum()/ (A**2).sum()) if with_scale else 1.0
    yaw = math.degrees(math.atan2(R[1,0], R[0,0]))
    res = dst - (s * (src-sc) @ R.T + dc)
    return yaw, s, np.linalg.norm(res, axis=1)

for name in ["go2_loop1_0812_1440", "go2_loop1_0812_1449"]:
    d = os.path.join(BASE, name)
    if not os.path.isdir(d): print(f"[없음] {name}"); continue
    odo, gps = load(d)
    print(f"\n{'='*52}\n{name}")
    if len(gps) < 10: print("  GPS fix 부족"); continue

    P = enu(gps)

    # --- GPS 원시 폐루프 오차 (앞뒤 10초 평균) ---
    t = gps[:,0] - gps[0,0]
    h, t_ = P[t <= 10], P[t >= t[-1]-10]
    gps_err = np.linalg.norm(t_.mean(0) - h.mean(0))
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    print(f"  GPS 경로장 {seg.sum():.1f}m,  hdop평균 {gps[:,3].mean():.2f}")
    print(f"  ** GPS 원시 폐루프 오차 = {gps_err:.2f} m **")

    # --- 오도메트리를 GPS 시각에 보간 ---
    ox = np.interp(gps[:,0], odo[:,0], odo[:,1])
    oy = np.interp(gps[:,0], odo[:,0], odo[:,2])
    O = np.column_stack([ox, oy])
    o_err = np.linalg.norm(O[-1]-O[0])
    o_len = np.linalg.norm(np.diff(O,axis=0),axis=1).sum()
    print(f"  오도 경로장 {o_len:.1f}m,  폐루프 오차 {o_err:.2f} m ({o_err/o_len*100:.2f}%)")

    # --- 정렬 ---
    for label, ws in [("yaw만", False), ("yaw+스케일", True)]:
        yaw, s, res = fit(O, P, ws)
        extra = f", 스케일 {s:.4f}" if ws else ""
        print(f"  [{label}] 오도 +x 방위 = ENU {yaw:+.2f}deg{extra}")
        print(f"           잔차 RMS {np.sqrt((res**2).mean()):.2f}m  max {res.max():.2f}m")

    # --- 방위 해석 ---
    yaw, _, _ = fit(O, P, False)
    print(f"  → 오도 -y 방향은 방위각 {(90-yaw+180)%360:.1f}deg (0=북,180=남)")
