#!/usr/bin/env python3
# consolidate_params.py — 설정을 yaml 한 곳으로 (값은 그대로)
# 사용: python3 consolidate_params.py           # 미리보기 (diff만 출력)
#       python3 consolidate_params.py --apply   # 백업 후 적용
import ast, difflib, os, re, shutil, sys, time
import yaml

APPLY = '--apply' in sys.argv
H = os.path.expanduser
WS = H('~/catkin_point_lio_unilidar/src/point_lio_ros2')
LAUNCH = f'{WS}/launch/mapping_go2_fix.launch.py'
YAMLS = [f'{WS}/config/go2_fix.yaml', f'{WS}/config/go2_fix_leg_v1.yaml']
FIX = H('~/fastlio_ws/tools/l1_imu_fix.py')
ZVD = f'{WS}/scripts/zvd_node.py'
BAK = H('~/patch_backups_0930')
new = {}

def read(p):
    return open(p, encoding='utf-8').read()

def ylit(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, (int, float)):
        return repr(v)
    return "'" + str(v) + "'"

def node_name(src, path):
    m = (re.search(r"super\(\)\.__init__\(\s*['\"]([^'\"]+)['\"]", src)
         or re.search(r"Node\.__init__\(\s*self\s*,\s*['\"]([^'\"]+)['\"]", src))
    if not m:
        sys.exit(f'✗ 노드 이름을 못 찾음: {path}')
    return m.group(1)

def declared(src):
    out = {}
    for mm in re.finditer(r"declare_parameter\(\s*'(\w+)'\s*,\s*([^)]+?)\s*\)", src):
        try:
            out[mm.group(1)] = ast.literal_eval(mm.group(2))
        except Exception:
            pass
    return out

# 1) 런치 dict → yaml
src = read(LAUNCH)
pat = re.compile(r"^[ \t]*'([A-Za-z_]\w*)'[ \t]*:[ \t]*([^,#\n]+?)[ \t]*,?[ \t]*(#[^\n]*)?\n", re.M)
moved = {m.group(1): ast.literal_eval(m.group(2)) for m in pat.finditer(src)}
if moved:
    out = pat.sub('', src)
    out = re.sub(r',\s*\{\s*\}', '', out)
    out = re.sub(r'\{\s*\}\s*,\s*', '', out)
    compile(out, LAUNCH, 'exec')
    new[LAUNCH] = out
    print(f'[launch] yaml로 옮길 파라미터 {len(moved)}개: ' + ', '.join(f'{k}={v!r}' for k, v in moved.items()))
else:
    print('[launch] 옮길 파라미터 없음 (이미 적용됨)')

# 2) l1_imu_fix.py: ALPHA_BETA → 파라미터, 레버암 스위치
src = read(FIX)
fix_name = node_name(src, FIX)
out = src
m = re.search(r"^([ \t]*)self\.ALPHA_BETA\s*=\s*([0-9.]+)[^\n]*$", src, re.M)
if m:
    ind, val = m.group(1), float(m.group(2))
    block = '\n'.join([
        f"{ind}self.declare_parameter('alpha_beta', {val!r})        # 각가속도 EMA 계수 (클수록 부드러움)",
        f"{ind}self.declare_parameter('lever_centri_en', True)    # 원심항 ω×(ω×r) 적용",
        f"{ind}self.declare_parameter('lever_tangent_en', True)   # 접선항 α×r 적용",
        f"{ind}self.ALPHA_BETA = float(self.get_parameter('alpha_beta').value)",
        f"{ind}self.lever_centri_en = bool(self.get_parameter('lever_centri_en').value)",
        f"{ind}self.lever_tangent_en = bool(self.get_parameter('lever_tangent_en').value)",
        f"{ind}self.get_logger().info('lever arm: centri=%s tangent=%s alpha_beta=%.2f' % (self.lever_centri_en, self.lever_tangent_en, self.ALPHA_BETA))"])
    out = src[:m.start()] + block + src[m.end():]
    for term, flag in (('a_centri', 'lever_centri_en'), ('a_tangent', 'lever_tangent_en')):
        p = re.compile(rf"^([ \t]*)acc_lidar = acc_lidar \+ {term}[ \t]*$", re.M)
        if len(p.findall(out)) != 1:
            sys.exit(f'✗ l1_imu_fix.py: "acc_lidar + {term}" 줄이 정확히 1개가 아님')
        out = p.sub(lambda mm, t=term, f=flag: f"{mm.group(1)}if self.{f}:\n{mm.group(1)}    acc_lidar = acc_lidar + {t}", out)
    compile(out, FIX, 'exec')
    new[FIX] = out
    print(f'[l1_imu_fix] ALPHA_BETA={val} → alpha_beta, 레버암 스위치 2개 (기본 켬), 노드 이름 {fix_name}')
else:
    print('[l1_imu_fix] ALPHA_BETA 상수 없음 (이미 적용됨)')
fix_params = declared(out)

# 3) zvd_node.py: v_th / dwell / release → 파라미터
src = read(ZVD)
zvd_name = node_name(src, ZVD)
out = src
p = re.compile(r"^([ \t]*)self\.(v_th|dwell|release)\s*=\s*([-+]?[0-9.]+(?:[eE][-+]?\d+)?)[ \t]*(#[^\n]*)?$", re.M)
hits = list(p.finditer(src))
if hits:
    sup = re.search(r"super\(\)\.__init__|Node\.__init__", src)
    if not sup or sup.start() > hits[0].start():
        sys.exit('✗ zvd_node.py: Node 초기화가 상수보다 뒤에 있음 — 수동 확인 필요')
    def rep(mm):
        ind, name, val, cm = mm.group(1), mm.group(2), float(mm.group(3)), (mm.group(4) or '')
        return (f"{ind}self.declare_parameter('{name}', {val!r})  {cm}".rstrip() + '\n'
                + f"{ind}self.{name} = float(self.get_parameter('{name}').value)")
    out = p.sub(rep, src)
    compile(out, ZVD, 'exec')
    new[ZVD] = out
    print('[zvd_node] 상수 → 파라미터: ' + ', '.join(f'{h.group(2)}={float(h.group(3))!r}' for h in hits) + f', 노드 이름 {zvd_name}')
else:
    print('[zvd_node] 옮길 상수 없음 (이미 적용됨)')
zvd_params = declared(out)

# 4) yaml: /** 에 런치 값 추가 + 노드별 섹션
for yp in YAMLS:
    src = read(yp)
    data = yaml.safe_load(src)
    rp = data['/**']['ros__parameters']
    clash = {k for k in moved if k in rp and rp[k] != moved[k]}
    if clash:
        sys.exit(f'✗ {os.path.basename(yp)}: 이미 다른 값으로 있음 {clash}')
    add = {k: v for k, v in moved.items() if k not in rp}
    out = src
    if add:
        lines = src.split('\n')
        i = next(n for n, l in enumerate(lines) if l.strip() == 'ros__parameters:')
        ind = ' ' * (len(lines[i]) - len(lines[i].lstrip()) + 4)
        lines[i + 1:i + 1] = [f'{ind}# ── 런치 파일에서 옮김 (2026-09-30, 값 동일) ──'] + [f'{ind}{k}: {ylit(v)}' for k, v in add.items()]
        out = '\n'.join(lines)
    for name, params in ((fix_name, fix_params), (zvd_name, zvd_params)):
        key = '/' + name
        if params and key not in data:
            out = out.rstrip('\n') + f'\n\n{key}:\n    ros__parameters:\n' + ''.join(f'        {k}: {ylit(v)}\n' for k, v in params.items())
    chk = yaml.safe_load(out)
    for k, v in moved.items():
        got = chk['/**']['ros__parameters'].get(k)
        if got != v or type(got) != type(v):
            sys.exit(f'✗ {os.path.basename(yp)}: {k} 값/타입 불일치 {got!r} vs {v!r}')
    if out != src:
        new[yp] = out
    print(f'[{os.path.basename(yp)}] /** 에 {len(add)}개 추가 · 노드 섹션 /{fix_name}, /{zvd_name}')

if not new:
    sys.exit('\n바꿀 것이 없습니다 (이미 적용됨).')
for p_, t in new.items():
    print('\n' + '\n'.join(difflib.unified_diff(read(p_).splitlines(), t.splitlines(), p_, p_ + ' (새)', lineterm='', n=1)))
if APPLY:
    os.makedirs(BAK, exist_ok=True)
    stamp = time.strftime('%H%M%S')
    for p_, t in new.items():
        shutil.copy2(p_, f'{BAK}/{os.path.basename(p_)}.{stamp}')
        open(p_, 'w', encoding='utf-8').write(t)
    print(f'\n적용 완료 · 백업: {BAK}')
else:
    print('\n미리보기만 했습니다. 적용하려면 --apply')
