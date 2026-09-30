#!/usr/bin/env python3
# check_v1.py — 실기 전 v1 설정 점검 (읽기만 함)
import os, sys, filecmp, difflib
import yaml
import numpy as np

CFG = os.path.expanduser('~/catkin_point_lio_unilidar/src/point_lio_ros2/config')
cur, v1 = os.path.join(CFG, 'go2_fix.yaml'), os.path.join(CFG, 'go2_fix_leg_v1.yaml')
ok = lambda b: '✓' if b else '✗'

same = filecmp.cmp(cur, v1, shallow=False)
print(f'[yaml] go2_fix.yaml == go2_fix_leg_v1.yaml : {ok(same)}')
if not same:
    for line in difflib.unified_diff(open(v1).read().splitlines(), open(cur).read().splitlines(),
                                     'v1', 'current', lineterm='', n=0):
        print('   ', line)

rp = yaml.safe_load(open(cur))['/**']['ros__parameters']
EXPECT = {'zupt_en': True, 'zupt_flag_topic': '/zupt_active', 'leg_en': True,
          'leg_odom_topic': '/utlidar/robot_odom', 'leg_scale': 1.23, 'leg_cov': 0.01,
          'leg_rate_hz': 20.0, 'leg_vel_only': True, 'leg_omg_en': True,
          'leg_att_en': False, 'leg_use_z': False}
print('\n[파라미터]  이름 / 값 / 타입 / v1 기대값')
for k, v in rp.items():
    if not k.startswith(('zupt', 'leg')):
        continue
    exp = EXPECT.get(k)
    mark = '' if exp is None else ok(v == exp and type(v) == type(exp))
    print(f'  {k:16s} {str(v):44s} {type(v).__name__:6s} {"" if exp is None else exp} {mark}')

def find(d, key):
    if isinstance(d, dict):
        for k, v in d.items():
            if k == key:
                return v
            r = find(v, key)
            if r is not None:
                return r
print('\n[입력]  ' + ' · '.join(f'{k}={find(rp, k)}' for k in ('imu_topic', 'lid_topic', 'lidar_type', 'use_imu_as_input')))

sys.path.insert(0, os.path.expanduser('~/fastlio_ws/tools'))
import go2_calib as c
R = np.array(c.R_LB, float)
if 'leg_R_ib' in rp and 'leg_lever' in rp:
    dR = np.abs(np.array(rp['leg_R_ib'], float).reshape(3, 3) - R).max()
    dL = np.abs(np.array(rp['leg_lever'], float) - np.array(c.LEVER, float)).max()
    print(f'\n[go2_calib 대조]  leg_R_ib−R_LB 최대차 {dR:.1e} {ok(dR < 1e-5)} · leg_lever−LEVER 최대차 {dL:.1e} {ok(dL < 1e-6)}')
else:
    print('\n[go2_calib 대조]  leg_R_ib / leg_lever 가 yaml에 없음 ✗')
print(f'                  R_LB[0,0] = {R[0, 0]:+.6f} · K_OUTDOOR = {getattr(c, "K_OUTDOOR", "?")} · leg_scale = {rp.get("leg_scale")}')
