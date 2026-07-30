import sqlite3, struct, sys, csv

def parse(buf):
    le = buf[1] == 1
    p = [4]
    def align(n):
        o = (p[0]-4) % n
        if o: p[0] += n-o
    def u32():
        align(4); v = struct.unpack_from('<I' if le else '>I', buf, p[0])[0]; p[0]+=4; return v
    def i32():
        align(4); v = struct.unpack_from('<i' if le else '>i', buf, p[0])[0]; p[0]+=4; return v
    def f64n(n):
        align(8); v = struct.unpack_from(('<' if le else '>')+str(n)+'d', buf, p[0]); p[0]+=8*n; return v
    def s():
        n = u32(); v = buf[p[0]:p[0]+n-1].decode('utf-8','replace'); p[0]+=n; return v
    sec = i32(); nsec = u32(); s(); s()
    pos = f64n(3); quat = f64n(4)
    return sec + nsec*1e-9, pos, quat

db, out = sys.argv[1], sys.argv[2]
con = sqlite3.connect(db)
topic = sys.argv[3] if len(sys.argv) > 3 else '/aft_mapped_to_init'
row = con.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
if row is None:
    sys.exit('토픽 %s 없음. ros2 bag info 로 확인하세요.' % topic)
tid = row[0]
with open(out, 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['t','x','y','z','qx','qy','qz','qw'])
    n = 0
    for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        t, pos, q = parse(blob)
        w.writerow([f'{t:.6f}', *[f'{v:.6f}' for v in pos], *[f'{v:.6f}' for v in q]]); n += 1
print(out, n, 'rows')
