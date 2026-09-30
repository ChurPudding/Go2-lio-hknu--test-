#!/usr/bin/env python3
# 다리 샘플 시각 정렬: header stamp 링버퍼 → time_current+leg_delay 최근접 샘플, 필터 시각 기준 rate 제한, 로그에 mean(t_leg-t_filter)
import re, sys, pathlib
PKG = pathlib.Path.home() / 'catkin_point_lio_unilidar/src/point_lio_ros2'; S = PKG / 'src'; L = S / 'laserMapping.cpp'
if L.read_text().count('time_current') == 0: sys.exit('[FAIL] laserMapping.cpp에 time_current 가 없음 — 필터 시각 변수 이름 확인 필요')
def sub(path, pat, fn, tag):
    s = path.read_text(); new, n = re.subn(pat, fn, s, flags=re.M)
    if n != 1: sys.exit(f'[FAIL] {path.name} ({tag}): 매치 {n}개')
    path.with_name(path.name + '.bak_align').write_text(s); path.write_text(new); print(f'[ok] {path.name}: {tag}')
def after(path, anchor, lines, tag):
    sub(path, anchor, lambda m: m.group(0) + ''.join('\n' + re.match(r'[ \t]*', m.group(0)).group(0) + l for l in lines), tag)

after(L, r'^[ \t]*std::atomic<uint32_t> leg_seq\{0\};[^\n]*',
      ['#include <mutex>', '#include <deque>', '#include <algorithm>',
       'struct LegSample { double t, vx, vy, vz; };',
       'std::deque<LegSample> leg_buf;  std::mutex leg_mtx;   // 다리 샘플 링버퍼 (header stamp)'], 'globals')
after(L, r'^[ \t]*leg_seq\.fetch_add\(1\);[^\n]*',
      ['{ std::lock_guard<std::mutex> lk(leg_mtx);',
       '  leg_buf.push_back({m->header.stamp.sec + m->header.stamp.nanosec * 1e-9, m->twist.twist.linear.x, m->twist.twist.linear.y, m->twist.twist.linear.z});',
       '  if (leg_buf.size() > 300) leg_buf.pop_front(); }'], 'callback push')

s = L.read_text(); start = s.find('if (leg_en && !zupt_active.load()) {')
if start < 0: sys.exit('[FAIL] 호출부 블록을 못 찾음')
i = s.find('{', start); depth = 0; j = i
while j < len(s):
    if s[j] == '{': depth += 1
    elif s[j] == '}':
        depth -= 1
        if depth == 0: break
    j += 1
ind = re.match(r'[ \t]*', s[s.rfind('\n', 0, start) + 1:]).group(0); I = ind + '    '
NEW = '\n'.join([
 ind + 'if (leg_en && !zupt_active.load()) {',
 I + '// 다리 샘플을 필터 시각(time_current + leg_delay)에 최근접으로 선택, 필터 시각 기준 leg_rate_hz 이하 1회 갱신',
 I + 'static double leg_t_used = -1.0, leg_t_last = -1e9;  static int leg_n = 0;  static double leg_r2 = 0.0, leg_dt_sum = 0.0;',
 I + 'LegSample ls{};  bool have = false;',
 I + '{   std::lock_guard<std::mutex> lk(leg_mtx);',
 I + '    if (!leg_buf.empty()) {',
 I + '        const double tq = time_current + leg_delay;',
 I + '        const auto it = std::min_element(leg_buf.begin(), leg_buf.end(),',
 I + '            [&](const LegSample &a, const LegSample &b){ return std::fabs(a.t - tq) < std::fabs(b.t - tq); });',
 I + '        ls = *it;  have = std::fabs(ls.t - tq) < 0.05;   // 50 ms 이내 샘플만',
 I + '    }',
 I + '}',
 I + 'if (have && ls.t != leg_t_used && time_current - leg_t_last >= 1.0 / leg_rate_hz) {',
 I + '    leg_t_used = ls.t;  leg_t_last = time_current;',
 I + '    Eigen::Matrix<double,3,1> vleg = leg_Rib * (Eigen::Vector3d(ls.vx, ls.vy, ls.vz) * leg_scale);   // z = R_LB * k * v_leg',
 I + '    const double rn = kf_output.update_leg(vleg, leg_use_z, leg_cov, leg_att_en, leg_rL, leg_omg_en);',
 I + '    leg_r2 += rn * rn;  leg_dt_sum += ls.t - time_current;',
 I + '    if (++leg_n % 100 == 0) {',
 I + '        RCLCPP_INFO(rclcpp::get_logger("laserMapping"), "leg update #%d  z(IMU)=[%.2f %.2f %.2f]  innov RMS(100)=%.3f m/s  mean(t_leg-t_filter)=%+.3f s",',
 I + '                    leg_n, vleg(0), vleg(1), vleg(2), std::sqrt(leg_r2 / 100.0), leg_dt_sum / 100.0);',
 I + '        leg_r2 = 0.0;  leg_dt_sum = 0.0;',
 I + '    }',
 I + '}',
 ind + '}'])
L.with_name(L.name + '.bak_align2').write_text(s); L.write_text(s[:start] + NEW + s[j + 1:]); print('[ok] laserMapping.cpp: 호출부 블록 교체')

after(S/'parameters.h',   r'^[ \t]*extern bool leg_omg_en;[^\n]*', ['extern double leg_delay;   // 다리 샘플 시각 보정 [s]'], 'extern')
after(S/'parameters.cpp', r'^[ \t]*bool leg_omg_en = false;[^\n]*', ['double leg_delay = 0.0;'], 'global')
after(S/'parameters.cpp', r'^[ \t]*nh->declare_parameter<bool>\("leg_omg_en"[^\n]*', ['nh->declare_parameter<double>("leg_delay", 0.0);'], 'declare')
after(S/'parameters.cpp', r'^[ \t]*nh->get_parameter\("leg_omg_en"[^\n]*', ['nh->get_parameter("leg_delay", leg_delay);'], 'get')
Y = PKG / 'config/go2_fix.yaml'
after(Y, r'^[ \t]*leg_omg_en:[^\n]*', ['leg_delay: 0.0'], 'yaml')
y = Y.read_text(); Y.write_text(re.sub(r'^([ \t]*)leg_omg_en:.*$', r'\1leg_omg_en: true', y, flags=re.M)); print('[ok] go2_fix.yaml: leg_omg_en -> true')
