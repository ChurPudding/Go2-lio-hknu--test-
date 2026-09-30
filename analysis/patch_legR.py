#!/usr/bin/env python3
# leg_R_ib (base_link -> IMU 회전) 파라미터 삽입: parameters.h/.cpp, laserMapping.cpp, go2_fix.yaml
import re, sys, pathlib
PKG = pathlib.Path.home() / 'catkin_point_lio_unilidar/src/point_lio_ros2'

def ins_after(path, anchor, new_lines, indent_re=None):
    s = path.read_text()
    if new_lines[0].strip() in s:
        print(f'[skip] {path.name}: 이미 있음'); return
    m = list(re.finditer(anchor, s, re.M))
    if len(m) != 1:
        sys.exit(f'[FAIL] {path.name}: 앵커 {anchor!r} 매치 {len(m)}개 (1개여야 함) - 이 파일 해당 부분을 보여주세요')
    if indent_re and (mi := re.search(indent_re, s, re.M)):
        ind = mi.group(1)
    else:
        ind = re.match(r'[ \t]*', s[s.rfind('\n', 0, m[0].start()) + 1:]).group(0)
    end = s.find('\n', m[0].end())
    path.with_name(path.name + '.bak_legR').write_text(s)
    path.write_text(s[:end] + ''.join('\n' + ind + l for l in new_lines) + s[end:])
    print(f'[ok] {path.name}: {len(new_lines)}줄 삽입')

S = PKG / 'src'
ins_after(S/'parameters.h', r'^extern std::string leg_odom_topic;',
          ['extern std::vector<double> leg_R_ib;   // base_link -> IMU(L1) 회전, row-major 9개'])
ins_after(S/'parameters.cpp', r'^std::string leg_odom_topic\s*=',
          ['std::vector<double> leg_R_ib{1,0,0, 0,1,0, 0,0,1};'])
ins_after(S/'parameters.cpp', r'declare_parameter<std::string>\("leg_odom_topic"',
          ['nh->declare_parameter<std::vector<double>>("leg_R_ib", leg_R_ib);'])
ins_after(S/'parameters.cpp', r'get_parameter\("leg_odom_topic"',
          ['nh->get_parameter("leg_R_ib", leg_R_ib);'])
ins_after(S/'laserMapping.cpp', r'^std::atomic<double> leg_vx',
          ['Eigen::Matrix3d leg_Rib = Eigen::Matrix3d::Identity();   // base_link -> IMU(L1), yaml leg_R_ib'])
ins_after(S/'laserMapping.cpp', r'readParameters\(nh\);',
          ['if (leg_R_ib.size() == 9) leg_Rib = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(leg_R_ib.data());',
           'else RCLCPP_ERROR(nh->get_logger(), "leg_R_ib needs 9 values (got %zu) - using identity", leg_R_ib.size());',
           'RCLCPP_INFO(nh->get_logger(), "leg_R_ib[0,0] = %+.6f (expect +0.523029)", leg_Rib(0,0));'])
ins_after(S/'laserMapping.cpp', r'^[ \t]*.*leg_vz\.load\(\).*\);[ \t]*$',
          ['vleg = leg_Rib * vleg;   // z = R_LB * (k * v_leg): base_link -> IMU 프레임'],
          indent_re=r'^([ \t]*)Eigen::Matrix<double,3,1>\s*vleg\(')
Y = PKG / 'config/go2_fix.yaml'
ins_after(Y, r'^[ \t]*leg_odom_topic:',
          ['leg_R_ib: [0.523029, -0.838576, 0.152420, -0.810712, -0.544668, -0.214668, 0.263034, -0.011292, -0.964721]'])
y = Y.read_text(); Y.write_text(re.sub(r'^([ \t]*)leg_scale:.*$', r'\1leg_scale: 0.95', y, flags=re.M))
print('[ok] go2_fix.yaml: leg_scale -> 0.95')
