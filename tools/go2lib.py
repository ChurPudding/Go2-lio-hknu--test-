"""go2lib.py -- Go2 rosbag(.db3) 오프라인 분석 공용 모듈

ROS2 없이 sqlite3 + 수동 CDR 역직렬화로 동작한다.
사용법:  from go2lib import *   ;  B = Bag('go2_run_full_0.db3')
"""
import sqlite3
import struct

import numpy as np

# /lowstate 의 IMUState 오프셋 (encapsulation 헤더 4바이트 이후 기준)
LS_QUAT, LS_GYRO, LS_ACC, LS_RPY = 24, 40, 52, 64


class CDR:
    """CDR 역직렬화. 각 원시 타입은 자기 크기에 맞춰 정렬된다."""

    def __init__(self, buf):
        self.b = buf
        self.le = buf[1] == 1
        self.p = 4

    def _al(self, n):
        o = (self.p - 4) % n
        if o:
            self.p += n - o

    def u8(self):
        v = self.b[self.p]
        self.p += 1
        return v

    def u32(self):
        self._al(4)
        v = struct.unpack_from('<I' if self.le else '>I', self.b, self.p)[0]
        self.p += 4
        return v

    def i32(self):
        self._al(4)
        v = struct.unpack_from('<i' if self.le else '>i', self.b, self.p)[0]
        self.p += 4
        return v

    def f64n(self, n):
        self._al(8)
        f = ('<' if self.le else '>') + str(n) + 'd'
        v = struct.unpack_from(f, self.b, self.p)
        self.p += 8 * n
        return v

    def string(self):
        n = self.u32()
        s = self.b[self.p:self.p + n - 1].decode('utf-8', 'replace')
        self.p += n
        return s

    def header(self):
        sec = self.i32()
        nsec = self.u32()
        return sec + nsec * 1e-9, self.string()


def _imu(buf):
    c = CDR(buf)
    t, fid = c.header()
    q = c.f64n(4); c.f64n(9)
    g = c.f64n(3); c.f64n(9)
    a = c.f64n(3); c.f64n(9)
    return t, q, g, a


def _odom(buf):
    c = CDR(buf)
    t, fid = c.header()
    c.string()
    p = c.f64n(3); q = c.f64n(4); c.f64n(36)
    lin = c.f64n(3); ang = c.f64n(3)
    return t, p, q, lin, ang


def _pc2(buf):
    c = CDR(buf)
    t, fid = c.header()
    c.u32(); w = c.u32()
    nf = c.u32()
    flds = []
    for _ in range(nf):
        nm = c.string(); off = c.u32(); dt = c.u8(); c.u32()
        flds.append((nm, off, dt))
    c.u8(); ps = c.u32(); c.u32()
    dl = c.u32()
    data = buf[c.p:c.p + dl]
    A = np.frombuffer(data, dtype=np.uint8).reshape(-1, ps)
    xyz = A[:, 0:12].copy().view(np.float32).reshape(-1, 3).astype(np.float64)
    return t, xyz, flds, ps


class Bag:
    def __init__(self, path):
        self.con = sqlite3.connect(path)
        self.topics = {n: (i, ty) for i, n, ty in
                       self.con.execute("SELECT id,name,type FROM topics")}

    def _rows(self, topic):
        tid = self.topics[topic][0]
        return self.con.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? "
            "ORDER BY timestamp", (tid,))

    def imu(self, topic='/utlidar/imu'):
        """헤더시각 t, 쿼터니언 q, 자이로 g, 가속도 a"""
        T, Q, G, A = [], [], [], []
        for _, b in self._rows(topic):
            t, q, g, a = _imu(b)
            T.append(t); Q.append(q); G.append(g); A.append(a)
        return (np.array(T), np.array(Q), np.array(G), np.array(A))

    def odom(self, topic='/utlidar/robot_odom'):
        T, P, Q = [], [], []
        for _, b in self._rows(topic):
            t, p, q, _, _ = _odom(b)
            T.append(t); P.append(p); Q.append(q)
        Q = np.array(Q)
        for i in range(1, len(Q)):          # 쿼터니언 부호 연속화
            if Q[i] @ Q[i - 1] < 0:
                Q[i] = -Q[i]
        return np.array(T), np.array(P), Q

    def lowstate(self, topic='/lowstate'):
        """헤더가 없으므로 bag 수신시각을 쓴다 (/utlidar/imu 대비 약 -8ms)"""
        T, Q, G, A = [], [], [], []
        for ts, b in self._rows(topic):
            T.append(ts * 1e-9)
            Q.append(struct.unpack_from('<4f', b, 4 + LS_QUAT))
            G.append(struct.unpack_from('<3f', b, 4 + LS_GYRO))
            A.append(struct.unpack_from('<3f', b, 4 + LS_ACC))
        return (np.array(T), np.array(Q), np.array(G), np.array(A))

    def clouds(self, topic='/utlidar/cloud'):
        out = []
        for _, b in self._rows(topic):
            t, xyz, _, _ = _pc2(b)
            out.append((t, xyz))
        return out

    def cloud_fields(self, topic='/utlidar/cloud'):
        for _, b in self._rows(topic):
            t, xyz, flds, ps = _pc2(b)
            return flds, ps, len(xyz)


# ---------- 기하 유틸 ----------
def q2R(q):
    q = np.asarray(q, float)
    if q.ndim == 1:
        x, y, z, w = q / np.linalg.norm(q)
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - z * w); R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w); R[:, 2, 1] = 2 * (y * z + x * w); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rot_align(a, b):
    """a 를 b 로 보내는 최소 회전"""
    a = np.asarray(a, float); b = np.asarray(b, float)
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    v = np.cross(a, b); s = np.linalg.norm(v); c = a @ b
    if s < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / s ** 2)


def yaw_of(Q):
    x, y, z, w = Q.T
    return np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def still_segments(t, g, a, win=125, gthr=0.08, astd=0.20, minlen=1.0):
    """정지 구간 탐지 -> [(i0,i1), ...]"""
    from numpy.lib.stride_tricks import sliding_window_view
    gn = np.linalg.norm(g, axis=1); an = np.linalg.norm(a, axis=1)
    ok = (sliding_window_view(gn, win).max(axis=1) < gthr) & \
         (sliding_window_view(an, win).std(axis=1) < astd)
    idx = np.where(ok)[0]
    segs = []
    if len(idx):
        s = p = idx[0]
        for i in idx[1:]:
            if i - p > 1:
                segs.append((s, p + win)); s = i
            p = i
        segs.append((s, p + win))
    return [x for x in segs if t[x[1] - 1] - t[x[0]] > minlen]


def fit_plane(P):
    c = P.mean(0)
    w, V = np.linalg.eigh((P - c).T @ (P - c))
    n = V[:, 0]
    return n, -n @ c


def ransac_plane(P, rng, th=0.02, it=2000, rmin=0.6, rmax=8.0):
    r = np.linalg.norm(P, axis=1)
    P = P[(r > rmin) & (r < rmax) & np.isfinite(r)]
    best = (0, None)
    for _ in range(it):
        j = rng.choice(len(P), 3, replace=False)
        p0, p1, p2 = P[j]
        n = np.cross(p1 - p0, p2 - p0); L = np.linalg.norm(n)
        if L < 1e-6:
            continue
        n /= L; d = -n @ p0
        s = (np.abs(P @ n + d) < th).sum()
        if s > best[0]:
            best = (s, (n, d))
    n, d = best[1]
    inl = np.abs(P @ n + d) < th
    n, d = fit_plane(P[inl])
    return n, d, P[inl], P


def kabsch(X, Y, w=None):
    """Y ~= R X 를 만족하는 R"""
    H = X.T @ Y if w is None else (X * w[:, None]).T @ Y
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    res = Y - (R @ X.T).T
    q = 1 - np.sqrt((res ** 2).sum(1).mean()) / np.sqrt((Y ** 2).sum(1).mean())
    return R, q


def rot_angle(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
