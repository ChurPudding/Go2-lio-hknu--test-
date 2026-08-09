# go2_description (ROS2 Humble)

Unitree Go2 URDF 패키지를 ROS1(catkin) → ROS2(ament_cmake)로 변환한 것입니다.
`robot_state_publisher`로 로봇 모델 TF를 발행하고 RViz2에서 확인할 수 있습니다.

## 1. 빌드

```bash
# 워크스페이스 src에 이 폴더(go2_description)를 넣은 뒤
cd ~/ros2_ws        # 본인 워크스페이스 경로로
colcon build --packages-select go2_description
source install/setup.bash
```

의존 패키지가 없다면:
```bash
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-xacro
```

## 2. 단독 시각화 테스트 (실로봇 없이)

```bash
ros2 launch go2_description display.launch.py
```

- 슬라이더 GUI로 다리 관절을 움직여볼 수 있습니다.
- RViz2에 로봇 모델과 TF 축이 뜨면 성공입니다.
- Fixed Frame 은 `base` 로 설정되어 있습니다.

## 3. 주요 프레임 (base 기준)

| child | xyz (m)                     | rpy (rad)         | 비고 |
|-------|-----------------------------|-------------------|------|
| radar | 0.28945, 0, -0.046825       | 0, 2.8782, 0      | **L1 LiDAR** (이름이 radar) |
| imu   | -0.02557, 0, 0.04232        | 0, 0, 0           | IMU |

> LiDAR가 pitch ≈ 2.8782 rad(약 165°) 기울어져 장착돼 있음. 포인트클라우드 정합 시 반드시 반영.

## 4. 실로봇 연동 시 (다음 단계)

- `use_gui:=false` 로 두고 실제 로봇의 `/joint_states`를 사용:
  ```bash
  ros2 launch go2_description display.launch.py use_gui:=false
  ```
- URDF 루트 링크 이름은 `base` 입니다. 실제 로봇 TF가 `base_link`를 쓴다면
  둘을 잇는 static transform이 필요합니다:
  ```bash
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base
  ```
- 이후 `/utlidar/robot_pose`(odom→base_link)와 합쳐지면 전체 TF 트리 완성.

## 5. 주의

- setup_go2.sh(CycloneDDS) 소싱 환경과 섞이지 않도록 터미널 관리 유의.
