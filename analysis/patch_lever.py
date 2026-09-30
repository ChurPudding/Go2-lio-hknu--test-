#!/usr/bin/env python3
# 레버암 항: update_leg(h = R̂ᵀv̂ + ⌊r_L⌋ₓω̂, δω 야코비안, 잔차 norm 반환) + leg_lever 파라미터 + 이노베이션 RMS 로그
import re, sys, pathlib
PKG = pathlib.Path.home() / 'catkin_point_lio_unilidar/src/point_lio_ros2'
def sub(path, pat, fn, tag):
    s = path.read_text(); new, n = re.subn(pat, fn, s, flags=re.M)
    if n != 1: sys.exit(f'[FAIL] {path.name} ({tag}): 매치 {n}개')
    path.with_name(path.name + '.bak_lever').write_text(s); path.write_text(new); print(f'[ok] {path.name}: {tag}')
def after(path, anchor, lines, tag):
    sub(path, anchor, lambda m: m.group(0) + ''.join('\n' + re.match(r'[ \t]*', m.group(0)).group(0) + l for l in lines), tag)

E = PKG / 'include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp'
sub(E, r'void update_leg\(', lambda m: 'scalar_type update_leg(', 'return type')
sub(E, r'bool att_en = true\)', lambda m: 'bool att_en = true,\n\t                const Eigen::Matrix<scalar_type,3,1> &r_lever = Eigen::Matrix<scalar_type,3,1>::Zero())', 'lever arg')
after(E, r'^[ \t]*Eigen::Matrix<scalar_type,3,1> Rtv = Rt \* v;[^\n]*', [
    'Eigen::Matrix<scalar_type,3,1> w(x_.omg[0], x_.omg[1], x_.omg[2]);   // ω̂ (L1 프레임, 상태 omg)',
    'Eigen::Matrix<scalar_type,3,3> skr;                                  // ⌊r_L⌋×',
    'skr <<           0, -r_lever(2),  r_lever(1),',
    '        r_lever(2),           0, -r_lever(0),',
    '       -r_lever(1),  r_lever(0),           0;',
    'Eigen::Matrix<scalar_type,3,1> h = Rtv + skr * w;                   // 예측 = base 원점 속도 = R̂ᵀv̂ − ω̂×r_L'], 'h with lever')
sub(E, r'r = \(v_leg_body - Rtv\)\.topRows\(md\);', lambda m: 'r = (v_leg_body - h).topRows(md);', 'residual')
after(E, r'^[ \t]*H\.block\(0,12, md, 3\) = Rt\.topRows\(md\);[^\n]*', ['H.block(0,15, md, 3) = skr.topRows(md);          // δω 자리 (레버암)'], 'H omega')
sub(E, r'^([ \t]*)Matrix<scalar_type, n, n> Ps = P_ \+ P_\.transpose\(\);[^\n]*\n[ \t]*x_\.boxplus\(dx_\);',
    lambda m: m.group(0) + '\n' + m.group(1) + 'return r.norm();', 'return')

S = PKG / 'src'
after(S/'parameters.h', r'^[ \t]*extern std::vector<double> leg_R_ib;[^\n]*', ['extern std::vector<double> leg_lever;   // base 원점 -> L1 원점 (base 프레임, m)'], 'extern')
after(S/'parameters.cpp', r'^[ \t]*std::vector<double> leg_R_ib\{[^\n]*', ['std::vector<double> leg_lever{0.0, 0.0, 0.0};'], 'global')
after(S/'parameters.cpp', r'^[ \t]*nh->declare_parameter<std::vector<double>>\("leg_R_ib"[^\n]*', ['nh->declare_parameter<std::vector<double>>("leg_lever", leg_lever);'], 'declare')
after(S/'parameters.cpp', r'^[ \t]*nh->get_parameter\("leg_R_ib"[^\n]*', ['nh->get_parameter("leg_lever", leg_lever);'], 'get')

L = S / 'laserMapping.cpp'
after(L, r'^[ \t]*Eigen::Matrix3d leg_Rib = [^\n]*', ['Eigen::Vector3d leg_rL = Eigen::Vector3d::Zero();   // r_L = R_LB * LEVER (L1 프레임)'], 'global rL')
after(L, r'^[ \t]*RCLCPP_INFO\(nh->get_logger\(\), "leg_R_ib\[0,0\][^\n]*', [
    'if (leg_lever.size() == 3) leg_rL = leg_Rib * Eigen::Vector3d(leg_lever[0], leg_lever[1], leg_lever[2]);',
    'RCLCPP_INFO(nh->get_logger(), "leg lever r_L (L1 frame) = [%.3f %.3f %.3f]", leg_rL(0), leg_rL(1), leg_rL(2));'], 'rL init')
sub(L, r'static int leg_n = 0;', lambda m: 'static int leg_n = 0;  static double leg_r2 = 0.0;', 'static r2')
sub(L, r'kf_output\.update_leg\(vleg, leg_use_z, leg_cov, leg_att_en\);',
    lambda m: 'const double rn = kf_output.update_leg(vleg, leg_use_z, leg_cov, leg_att_en, leg_rL);  leg_r2 += rn * rn;', 'call')
sub(L, r'RCLCPP_INFO\(rclcpp::get_logger\("laserMapping"\), "leg update #%d\s+z\(IMU\)=\[%.2f %.2f %.2f\]",\s*leg_n, vleg\(0\), vleg\(1\), vleg\(2\)\);',
    lambda m: '{ RCLCPP_INFO(rclcpp::get_logger("laserMapping"), "leg update #%d  z(IMU)=[%.2f %.2f %.2f]  innov RMS(100)=%.3f m/s", leg_n, vleg(0), vleg(1), vleg(2), std::sqrt(leg_r2 / 100.0));  leg_r2 = 0.0; }', 'log')

after(PKG/'config/go2_fix.yaml', r'^[ \t]*leg_att_en:[^\n]*', ['leg_lever: [0.322, 0.005, 0.05]'], 'yaml')
