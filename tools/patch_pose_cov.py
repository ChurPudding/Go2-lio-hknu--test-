#!/usr/bin/env python3
"""
patch_pose_cov.py -- robot_pose.py 가 /indoor/base_pose 하나에
                     위치와 신뢰도를 모두 담도록 고친다.

팀원이 받는 토픽을 최소화하기 위한 것이다.
health 를 별도 Bool 토픽으로 받지 않고 Odometry 의 covariance 로 표현한다.

  정상  pose.covariance[0]  = 0.01   (x 분산, 약 10 cm)
  이상  pose.covariance[0]  = 1e6    (사실상 무한 = 이 위치를 믿지 말 것)

covariance 는 ROS 표준 필드라 robot_localization·Nav2 가 자동으로 반영한다.
직접 만든 A* 라면 아래 한 줄만 확인하면 된다.

    if msg.pose.covariance[0] > 100:   # 위치 신뢰 불가 -> 정지
"""
import re, sys, shutil

p = '/home/hyo/fastlio_ws/tools/robot_pose.py'
s = open(p).read()

if 'health_cov' in s:
    print('이미 적용됨'); sys.exit()

shutil.copy(p, p + '.bak_cov')

# 1) import 보강
if 'from std_msgs.msg import Bool' not in s:
    s = s.replace('from nav_msgs.msg import Odometry',
                  'from nav_msgs.msg import Odometry\nfrom std_msgs.msg import Bool')

# 2) __init__ 에 health 구독 추가 (publisher 생성 직후)
m = re.search(r'^(\s*)self\.pub = self\.create_publisher\(\n?\s*Odometry.*?\)\n',
              s, re.M | re.S)
if not m:
    m = re.search(r'^(\s*)self\.pub = self\.create_publisher\([^\n]*\n', s, re.M)
if not m:
    print('✗ publisher 를 찾지 못했습니다. 수동 수정이 필요합니다.'); sys.exit(1)
ind = m.group(1)
ins = (f'{ind}# 신뢰도를 covariance 로 실어 보내기 위해 health 를 구독한다\n'
       f'{ind}self.health_cov = 0.01\n'
       f'{ind}self.declare_parameter("health_topic", "/indoor/health")\n'
       f'{ind}self.create_subscription(\n'
       f'{ind}    Bool, self.get_parameter("health_topic").value,\n'
       f'{ind}    lambda m: setattr(self, "health_cov", 0.01 if m.data else 1e6), 10)\n')
s = s[:m.end()] + ins + s[m.end():]

# 3) publish 직전에 covariance 채우기
m2 = re.search(r'^(\s*)self\.pub\.publish\(out\)', s, re.M)
if not m2:
    print('✗ publish(out) 를 찾지 못했습니다. 수동 수정이 필요합니다.'); sys.exit(1)
ind2 = m2.group(1)
cov = (f'{ind2}# 신뢰도: 정상 0.01, 이상 1e6 (표준 covariance 관례)\n'
       f'{ind2}c = self.health_cov\n'
       f'{ind2}out.pose.covariance = [c, 0.0, 0.0, 0.0, 0.0, 0.0,\n'
       f'{ind2}                       0.0, c, 0.0, 0.0, 0.0, 0.0,\n'
       f'{ind2}                       0.0, 0.0, c, 0.0, 0.0, 0.0,\n'
       f'{ind2}                       0.0, 0.0, 0.0, c, 0.0, 0.0,\n'
       f'{ind2}                       0.0, 0.0, 0.0, 0.0, c, 0.0,\n'
       f'{ind2}                       0.0, 0.0, 0.0, 0.0, 0.0, c]\n')
s = s[:m2.start()] + cov + s[m2.start():]

open(p, 'w').write(s)
print('적용 완료.  백업: robot_pose.py.bak_cov')
