#!/usr/bin/env python3
import re, sys, pathlib
PKG = pathlib.Path.home() / 'catkin_point_lio_unilidar/src/point_lio_ros2'
def edit(path, anchor, repl_or_lines, mode):
    s = path.read_text(); m = list(re.finditer(anchor, s, re.M | re.S))
    if len(m) != 1: sys.exit(f'[FAIL] {path.name}: {anchor!r} 매치 {len(m)}개')
    path.with_name(path.name + '.bak_legRate').write_text(s)
    if mode == 'replace':
        s = s[:m[0].start()] + repl_or_lines + s[m[0].end():]
    else:
        ind = re.match(r'[ \t]*', s[s.rfind('\n', 0, m[0].start()) + 1:]).group(0)
        end = s.find('\n', m[0].end()); s = s[:end] + ''.join('\n' + ind + l for l in repl_or_lines) + s[end:]
    path.write_text(s); print(f'[ok] {path.name}: {mode}')

E = PKG / 'include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp'
edit(E, r'bool use_z, scalar_type r_leg\)', 'bool use_z, scalar_type r_leg, bool att_en = true)', 'replace')
edit(E, r'H\.block\(0, 3, md, 3\) = sk\.topRows\(md\);', 'if (att_en) H.block(0, 3, md, 3) = sk.topRows(md);', 'replace')
edit(E, r'P_ -= K \* \(H \* P_\);', ['Matrix<scalar_type, n, n> Ps = P_ + P_.transpose();  P_ = scalar_type(0.5) * Ps;   // 대칭 유지'], 'after')

S = PKG / 'src'
edit(S/'parameters.h', r'^extern std::vector<double> leg_R_ib;.*$', ['extern double leg_rate_hz;  extern bool leg_att_en;'], 'after')
edit(S/'parameters.cpp', r'^std::vector<double> leg_R_ib\{.*$', ['double leg_rate_hz = 20.0;  bool leg_att_en = false;'], 'after')
edit(S/'parameters.cpp', r'declare_parameter<std::vector<double>>\("leg_R_ib".*$',
     ['nh->declare_parameter<double>("leg_rate_hz", 20.0);', 'nh->declare_parameter<bool>("leg_att_en", false);'], 'after')
edit(S/'parameters.cpp', r'get_parameter\("leg_R_ib".*$',
     ['nh->get_parameter("leg_rate_hz", leg_rate_hz);', 'nh->get_parameter("leg_att_en", leg_att_en);'], 'after')

L = S / 'laserMapping.cpp'
edit(L, r'^std::atomic<double> leg_vx.*$', ['#include <chrono>', 'std::atomic<uint32_t> leg_seq{0};   // 다리 샘플 카운터'], 'after')
edit(L, r'leg_vz\.store\(m->twist\.twist\.linear\.z\);', ['leg_seq.fetch_add(1);'], 'after')
edit(L, r'kf_output\.update_leg\(vleg, leg_use_z, leg_cov\);',
'''{   // 새 다리 샘플에 대해서만, leg_rate_hz 이하로 1회 갱신
                            static uint32_t leg_seq_used = 0;  static double leg_t_last = -1.0;  static int leg_n = 0;
                            const uint32_t sq = leg_seq.load();
                            const double tnow = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
                            if (sq != leg_seq_used && (leg_t_last < 0 || tnow - leg_t_last >= 1.0 / leg_rate_hz)) {
                                leg_seq_used = sq;  leg_t_last = tnow;
                                kf_output.update_leg(vleg, leg_use_z, leg_cov, leg_att_en);
                                if (++leg_n % 100 == 0)
                                    RCLCPP_INFO(rclcpp::get_logger("laserMapping"), "leg update #%d  z(IMU)=[%.2f %.2f %.2f]", leg_n, vleg(0), vleg(1), vleg(2));
                            }
                        }''', 'replace')

Y = PKG / 'config/go2_fix.yaml'
edit(Y, r'^[ \t]*leg_R_ib:.*$', ['leg_rate_hz: 20.0', 'leg_att_en: false'], 'after')
