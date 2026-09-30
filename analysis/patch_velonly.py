#!/usr/bin/env python3
# leg_vel_only: 이득을 pos/vel 행으로 제한 + Joseph 공분산 갱신 — update_leg 범위 안에서만, 재실행 안전
import re, sys, pathlib
PKG = pathlib.Path.home() / 'catkin_point_lio_unilidar/src/point_lio_ros2'; S = PKG / 'src'
E = PKG / 'include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp'

s = E.read_text(); m = re.search(r'update_leg\(', s)
if not m: sys.exit('[FAIL] update_leg 못 찾음')
start = m.start(); i = s.find('{', start); depth = 0; j = i
while j < len(s):
    if s[j] == '{': depth += 1
    elif s[j] == '}':
        depth -= 1
        if depth == 0: break
    j += 1
sig, body = s[start:i], s[i:j + 1]
if 'vel_only' not in sig:
    sig, n = re.subn(r'bool omg_en = false\)', 'bool omg_en = false, bool vel_only = false)', sig)
    if n != 1: sys.exit('[FAIL] 시그니처 앵커'); 
    print('[ok] esekfom.hpp: arg')
else: print('[skip] esekfom.hpp: arg 이미 있음')
if 'if (vel_only)' not in body:
    def f(mm):
        ind = re.match(r'[ \t]*', mm.group(0)).group(0)
        L = ['if (vel_only) {                                       // Schmidt식: pos(0-2)·vel(12-14) 행만 갱신',
             '    Matrix<scalar_type, n, Eigen::Dynamic> Kv = Matrix<scalar_type, n, Eigen::Dynamic>::Zero(n, md);',
             '    Kv.block(0, 0, 3, md)  = K.block(0, 0, 3, md);',
             '    Kv.block(12, 0, 3, md) = K.block(12, 0, 3, md);',
             '    K = Kv;',
             '}']
        return mm.group(0) + ''.join('\n' + ind + l for l in L)
    body, n = re.subn(r'^[ \t]*Matrix<scalar_type, n, Eigen::Dynamic> K = PHT \* HPHT\.inverse\(\);[^\n]*', f, body, flags=re.M)
    if n != 1: sys.exit(f'[FAIL] K 앵커(update_leg 안) 매치 {n}개')
    print('[ok] esekfom.hpp: K mask')
else: print('[skip] esekfom.hpp: K mask 이미 있음')
if 'Joseph' not in body:
    body, n = re.subn(r'^([ \t]*)P_ -= K \* \(H \* P_\);[^\n]*',
        lambda mm: mm.group(1) + 'Matrix<scalar_type, n, n> IKH = Matrix<scalar_type, n, n>::Identity() - K * H;\n'
                 + mm.group(1) + 'Matrix<scalar_type, n, n> Pj = IKH * P_ * IKH.transpose() + K * (r_leg * K.transpose());   // Joseph (임의 이득에 일관)\n'
                 + mm.group(1) + 'P_ = Pj;', body, flags=re.M)
    if n != 1: sys.exit(f'[FAIL] Joseph 앵커 매치 {n}개')
    print('[ok] esekfom.hpp: Joseph')
else: print('[skip] esekfom.hpp: Joseph 이미 있음')
E.with_name(E.name + '.bak_velonly2').write_text(s); E.write_text(s[:start] + sig + body + s[j + 1:])

def sub(path, pat, fn, tag, marker):
    s = path.read_text()
    if marker in s: print(f'[skip] {path.name}: {tag} 이미 있음'); return
    new, n = re.subn(pat, fn, s, flags=re.M)
    if n != 1: sys.exit(f'[FAIL] {path.name} ({tag}): 매치 {n}개')
    path.write_text(new); print(f'[ok] {path.name}: {tag}')
def after(path, anchor, lines, tag, marker):
    sub(path, anchor, lambda m: m.group(0) + ''.join('\n' + re.match(r'[ \t]*', m.group(0)).group(0) + l for l in lines), tag, marker)
sub(S/'laserMapping.cpp', r'leg_rL, leg_omg_en\)', lambda m: 'leg_rL, leg_omg_en, leg_vel_only)', 'call', 'leg_vel_only)')
after(S/'parameters.h',   r'^[ \t]*extern bool leg_omg_en;[^\n]*', ['extern bool leg_vel_only;'], 'extern', 'leg_vel_only')
after(S/'parameters.cpp', r'^[ \t]*bool leg_omg_en = false;[^\n]*', ['bool leg_vel_only = false;'], 'global', 'bool leg_vel_only')
after(S/'parameters.cpp', r'^[ \t]*nh->declare_parameter<bool>\("leg_omg_en"[^\n]*', ['nh->declare_parameter<bool>("leg_vel_only", false);'], 'declare', '"leg_vel_only", false')
after(S/'parameters.cpp', r'^[ \t]*nh->get_parameter\("leg_omg_en"[^\n]*', ['nh->get_parameter("leg_vel_only", leg_vel_only);'], 'get', 'get_parameter("leg_vel_only"')
Y = PKG / 'config/go2_fix.yaml'
after(Y, r'^[ \t]*leg_omg_en:[^\n]*', ['leg_vel_only: true'], 'yaml', 'leg_vel_only')
y = Y.read_text(); Y.write_text(re.sub(r'^([ \t]*)leg_cov:.*$', r'\1leg_cov: 0.01', y, flags=re.M)); print('[ok] go2_fix.yaml: leg_cov -> 0.01')
