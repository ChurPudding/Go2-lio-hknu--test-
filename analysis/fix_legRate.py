#!/usr/bin/env python3
import re, sys, pathlib
PKG = pathlib.Path.home() / 'catkin_point_lio_unilidar/src/point_lio_ros2'

def move(path, misplaced, anchor):
    lines = path.read_text().split('\n')
    for txt in misplaced:                      # 잘못 붙은 줄: 마지막 출현을 제거
        idx = [i for i, l in enumerate(lines) if l.strip() == txt.strip()]
        if not idx: sys.exit(f'[FAIL] {path.name}: 못 찾음 -> {txt[:40]}')
        lines.pop(idx[-1])
    hits = [i for i, l in enumerate(lines) if re.search(anchor, l)]
    if len(hits) != 1: sys.exit(f'[FAIL] {path.name}: 앵커 {anchor!r} {len(hits)}개')
    ind = re.match(r'[ \t]*', lines[hits[0]]).group(0)
    for k, txt in enumerate(misplaced):        # 앵커 줄 바로 뒤에 재삽입
        lines.insert(hits[0] + 1 + k, ind + txt.strip())
    path.write_text('\n'.join(lines)); print(f'[ok] {path.name}: {len(misplaced)}줄 이동')

S = PKG / 'src'
move(S/'parameters.h', ['extern double leg_rate_hz;  extern bool leg_att_en;'], r'^extern std::vector<double> leg_R_ib;')
move(S/'parameters.cpp', ['double leg_rate_hz = 20.0;  bool leg_att_en = false;'], r'^std::vector<double> leg_R_ib\{')
move(S/'parameters.cpp', ['nh->declare_parameter<double>("leg_rate_hz", 20.0);', 'nh->declare_parameter<bool>("leg_att_en", false);'],
     r'declare_parameter<std::vector<double>>\("leg_R_ib"')
move(S/'parameters.cpp', ['nh->get_parameter("leg_rate_hz", leg_rate_hz);', 'nh->get_parameter("leg_att_en", leg_att_en);'],
     r'get_parameter\("leg_R_ib"')
move(S/'laserMapping.cpp', ['#include <chrono>', 'std::atomic<uint32_t> leg_seq{0};   // 다리 샘플 카운터'], r'^std::atomic<double> leg_vx')
move(PKG/'config/go2_fix.yaml', ['leg_rate_hz: 20.0', 'leg_att_en: false'], r'^[ \t]*leg_R_ib:')
