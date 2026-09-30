#!/usr/bin/env python3
# δω 야코비안 블록을 leg_omg_en 스위치로 (기본 false): 레버암은 h의 알려진 보정으로만
import re, sys, pathlib
PKG = pathlib.Path.home() / 'catkin_point_lio_unilidar/src/point_lio_ros2'
def sub(path, pat, fn, tag):
    s = path.read_text(); new, n = re.subn(pat, fn, s, flags=re.M)
    if n != 1: sys.exit(f'[FAIL] {path.name} ({tag}): 매치 {n}개')
    path.with_name(path.name + '.bak_omg').write_text(s); path.write_text(new); print(f'[ok] {path.name}: {tag}')
def after(path, anchor, lines, tag):
    sub(path, anchor, lambda m: m.group(0) + ''.join('\n' + re.match(r'[ \t]*', m.group(0)).group(0) + l for l in lines), tag)
E = PKG / 'include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp'; S = PKG / 'src'
sub(E, r'Eigen::Matrix<scalar_type,3,1>::Zero\(\)\)', lambda m: 'Eigen::Matrix<scalar_type,3,1>::Zero(), bool omg_en = false)', 'omg arg')
sub(E, r'H\.block\(0,15, md, 3\) = skr\.topRows\(md\);', lambda m: 'if (omg_en) H.block(0,15, md, 3) = skr.topRows(md);', 'H omega gate')
sub(S/'laserMapping.cpp', r'update_leg\(vleg, leg_use_z, leg_cov, leg_att_en, leg_rL\)', lambda m: 'update_leg(vleg, leg_use_z, leg_cov, leg_att_en, leg_rL, leg_omg_en)', 'call')
after(S/'parameters.h', r'^[ \t]*extern double leg_rate_hz;[^\n]*', ['extern bool leg_omg_en;'], 'extern')
after(S/'parameters.cpp', r'^[ \t]*double leg_rate_hz = 20\.0;[^\n]*', ['bool leg_omg_en = false;'], 'global')
after(S/'parameters.cpp', r'^[ \t]*nh->declare_parameter<bool>\("leg_att_en"[^\n]*', ['nh->declare_parameter<bool>("leg_omg_en", false);'], 'declare')
after(S/'parameters.cpp', r'^[ \t]*nh->get_parameter\("leg_att_en"[^\n]*', ['nh->get_parameter("leg_omg_en", leg_omg_en);'], 'get')
after(PKG/'config/go2_fix.yaml', r'^[ \t]*leg_att_en:[^\n]*', ['leg_omg_en: false'], 'yaml')
