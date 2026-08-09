# 팀원용 코드 목록

`~/fastlio_ws` 저장소에 무엇이 있고 각각 언제 쓰는지 정리했습니다.
갱신 2026-08-10 · 담당 효신

> **2026-08-07 갱신**: 실내 지도 생성 방식이 Point-LIO 에서 다리
> 오도메트리 점군 누적으로 바뀌었습니다(같은 bag 에서 드리프트 21.75% →
> 2.63%). Point-LIO 관련 도구·설정은 삭제하지 않고 **보류**로 표시했습니다
> — 원인 미규명 상태로 남겨 둔 것일 뿐입니다. 상세 근거는
> `docs/2026-08-07_실험기록.md`.

---

## 한눈에

| 상황 | 실행할 것 |
|---|---|
| 처음 설치했다 | `tools/install_go2_lio.sh` → `tools/doctor.sh` |
| 로봇으로 주행하겠다 | `tools/run_indoor.sh` |
| 녹화본으로 시험하겠다 | `tools/run_indoor.sh bag <경로>` |
| 지도를 새로 만들겠다 | bag 녹화 → `tools/odom_map_build.py` → `tools/loop_correct_v2.py` → `tools/pcd_to_grid.py` |
| 지도를 토픽으로 받겠다 | `tools/map_publisher.py` |
| 뭔가 안 된다 | `tools/doctor.sh robot` |

**대부분의 경우 `run_indoor.sh` 하나면 됩니다.** 아래 목록은 그 안에서 무엇이
도는지, 문제가 생겼을 때 어디를 봐야 하는지 알기 위한 것입니다.

---

## A. 팀원이 직접 쓰는 것

### A-1. `tools/run_indoor.sh` — 파이프라인 실행

```bash
./tools/run_indoor.sh              # 실시간 (로봇 연결 필요)
./tools/run_indoor.sh bag <경로>    # 녹화본 재생
./tools/run_indoor.sh bag <경로> 0.25   # 재생 속도 지정 (기본 0.5)
```

노드 5개를 순서대로 띄우고, `Ctrl+C` 로 역순 종료합니다. 실시간 모드는 시작
전에 로봇 토픽 4개의 수신을 확인하고 하나라도 없으면 중단합니다.

로그는 `/tmp/go2_indoor/` 에 노드별로 쌓입니다.

```bash
tail -f /tmp/go2_indoor/health.log     # 위치 신뢰도
tail -f /tmp/go2_indoor/lio.log        # Point-LIO
tail -f /tmp/go2_indoor/pose.log       # 몸통 위치
```

**이 파이프라인은 실시간 위치추정·주행용입니다.** 지도 생성에는 더 이상
쓰지 않습니다 — 지도는 이 스크립트로 녹화한 bag 을 아래 A-4 절차로
오프라인 처리해서 만듭니다.

---

### A-2. `tools/doctor.sh` — 점검

```bash
./tools/doctor.sh          # 파일·설정 (로봇 없이)
./tools/doctor.sh robot    # 로봇 연결·토픽 레이트까지
```

6개 항목을 확인하고 실패마다 고치는 방법을 알려 줍니다. **설치 직후와 문제가
생겼을 때 가장 먼저 돌려 보십시오.**

로봇 항목만 실패하면 설치는 정상이고 케이블 문제입니다.

---

### A-3. `tools/install_go2_lio.sh` — 설치

```bash
./tools/install_go2_lio.sh          # 전체 (30분~1시간)
./tools/install_go2_lio.sh check    # 상태만
./tools/install_go2_lio.sh 4        # 4단계만 다시
```

apt → unitree_ros2 → Livox-SDK2 → livox_ros_driver2 → Point-LIO → 설정 순서로
진행합니다. 다시 돌려도 안전하며 끝난 단계는 건너뜁니다. `sudo` 비밀번호를
두 번 묻습니다.

---

### A-4. 지도 만들기 — 다리 오도메트리 점군 누적 (2026-08-07 전환)

**Point-LIO 대신 다리 오도메트리로 지도를 만듭니다.** 같은 bag 에서 드리프트가
Point-LIO 21.75% 대 다리 오도메트리 2.63%로 8배 이상 차이가 났기 때문입니다.
Point-LIO 경로는 원인 미규명 상태로 **보류**이며 코드는 지우지 않았습니다
(B 절 그대로 유효, run_indoor.sh 의 실시간 위치추정에는 계속 씁니다).
상세: `docs/2026-08-07_실험기록.md`.

```bash
# 1. 로봇으로 층을 한 바퀴 돌며 bag 녹화 (토픽 명시, /utlidar/cloud_deskewed 필수)
#    출발점에 정확히·같은 방향으로 복귀할 것 (목표 2 m 이내)

# 2. 점군 누적 — LIO 를 거치지 않고 bag 을 오프라인으로 직접 읽는다
python3 tools/odom_map_build.py <bag경로> 0.05

# 3. 루프 클로저 — 출발-도착 오차를 걸음마다 나눠 배분
python3 tools/loop_correct_v2.py <bag경로>

# 4. 2D 격자 변환 (기존과 동일한 도구)
python3 tools/pcd_to_grid.py results/odommap_v2/scans.pcd results/indoor_map 0.10
```

| 도구 | 하는 일 |
|---|---|
| `odom_map_build.py` | `/utlidar/cloud_deskewed`(이미 odom 좌표계)를 다리 오도메트리 자세로 그대로 누적. 변환·LIO 불필요 |
| `loop_correct_v2.py` | 증분 재적분 방식 루프 클로저. v1(`loop_correct.py`, 전역 나선 변환)은 회전 중심에서 먼 점을 크게 밀어내 45 m 로 튕겨나간 결함으로 **폐기**, 참고용으로만 남김 |
| `pcd_to_grid.py` | 3D 점군(PCD)을 A* 용 2D 점유격자로 변환. 입력 출처만 바뀌었을 뿐 도구 자체는 그대로 |

**`loop_correct_v2.py` 는 ICP `fitness ≥ 0.9` 일 때만 적용하십시오.** 낮으면
(예: 0.665) 보정이 오히려 지도를 비틉니다 — 이때는 `odom_map_build.py` 의
무보정 출력을 그대로 씁니다. 실행 로그에 fitness 값이 출력됩니다.

`pcd_to_grid.py` 인자:

| 인자 | 뜻 |
|---|---|
| 1 | PCD 경로 |
| 2 | 출력 접두사 |
| 3 | 격자 한 칸 크기 [m]. **0.10 을 쓰십시오** |

출력은 `.npy` `.pgm` `.yaml` `_preview.png` 네 개입니다.

**해상도 5 cm 는 쓰지 마십시오.** 점 밀도가 모자라 벽이 끊겨 보입니다. 같은
지도인데 "지도가 비었다"고 오판하기 쉽습니다.

**확인 방법**: 출력의 `z 범위`가 4 m 이내면 정상입니다. 12 m 처럼 크면 이전
회차 점군이 섞였거나 (Point-LIO 경로라면) LIO 가 발산한 것이니 PCD 를 지우고
다시 만드십시오.

> 2026-08-07 에 지면 추정 버그를 고쳤습니다. 최빈 높이 bin 의 **왼쪽 끝**이
> 아니라 **중앙**을 지면으로 잡습니다. bin 폭이 넓을수록 오차가 컸던
> 버그입니다 — 이미 반영돼 있으니 새로 받을 필요는 없습니다.

**높이 필터 기본값(`Z_MIN, Z_MAX = 0.20, 1.50`)은 5종 비교 후 그대로
유지합니다.** 범위를 넓히면 벽면이 서로 다른 높이에 어긋나게 쌓여 오히려
지도가 부서집니다. 자세한 비교 그림은 실험기록 6절 참고.

---

### A-5. `tools/map_publisher.py` — 지도를 토픽으로

```bash
python3 tools/map_publisher.py
python3 tools/map_publisher.py --ros-args -p yaml:=results/indoor_map.yaml
```

지도 파일을 `/indoor/map` (`nav_msgs/OccupancyGrid`) 으로 발행합니다.
파일로 직접 읽으실 거면 필요 없습니다.

**구독 시 QoS 를 `transient_local` 로 맞추셔야 합니다.** 지도는 한 번만
발행되고 붙잡혀 있어서, 기본 QoS 로는 아무것도 받지 못합니다.

---

### A-6. `examples/indoor_pose_subscriber/` — C++ 예제

바로 빌드되는 최소 패키지입니다. 본인 워크스페이스 `src/` 에 복사해 쓰십시오.

| 파일 | 내용 |
|---|---|
| `src/indoor_pose_subscriber.cpp` | 위치만 구독. 가장 단순 |
| `src/indoor_map_subscriber.cpp` | 위치 + 지도 구독. **주석 상세** |
| `src/map_loader.hpp` | 지도를 파일/토픽 양쪽으로 읽는 헬퍼 |

```bash
cp -r examples/indoor_pose_subscriber ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select indoor_pose_subscriber
source install/setup.bash
ros2 run indoor_pose_subscriber indoor_map_subscriber
```

`indoor_map_subscriber.cpp` 에 QoS·좌표변환·신뢰도검사가 왜 그렇게 되어 있는지
주석으로 다 적어 두었습니다. **이것부터 읽으시면 됩니다.**

---

## B. 파이프라인 내부 (직접 실행할 일은 거의 없음)

`run_indoor.sh` 가 자동으로 띄웁니다. 문제 원인을 찾을 때만 개별 실행합니다.

**이 절은 실시간 위치추정(주행) 파이프라인입니다.** Point-LIO 기반이며
현재도 그대로 씁니다. 지도 생성은 더 이상 여기에 의존하지 않습니다 (A-4 참고).

### B-1. `tools/l1_imu_fix.py` — IMU 보정

L1 라이다 내부 IMU 는 **164.9° 기울어 장착**돼 있습니다. 그대로 쓰면 중력
방향이 틀어져 LIO 가 즉시 발산합니다. 이 노드가 본체 IMU(`/lowstate`)의
가속도와 라이다 IMU(`/utlidar/imu`)의 자이로를 합쳐 `/l1_imu_fixed` 를 만듭니다.

**모든 LIO 실험의 전제입니다.** 이게 없으면 아무것도 안 됩니다.

### B-2. `tools/robot_pose.py` — 몸통 위치 변환

Point-LIO 가 내는 `/aft_mapped_to_init` 은 **라이다 위치**입니다. 몸통 중심은
그보다 32 cm 뒤에 있어서, 제자리에서 회전하면 라이다는 반지름 32 cm 원을
그립니다. 이 노드가 보정해 `/indoor/base_pose` 를 냅니다.

`/indoor/health` 를 구독해 신뢰도를 `pose.covariance[0]` 에 실어 보냅니다.

### B-3. `tools/lio_health.py` — 위치 신뢰도 감시

LIO 는 실패해도 조용히 틀린 좌표를 계속 냅니다. 네 가지를 감시합니다.

| 항목 | 내용 |
|---|---|
| 수신 끊김 | 0.5초 이상 무응답 |
| **정지 중 표류** | 로봇은 멈춰 있는데 LIO 위치가 움직임 |
| 속도 불일치 | LIO 속도와 로봇 자체 속도가 어긋남 |
| z 급변 | 평지인데 높이가 튐 |

복도 실패 데이터에서 **12.9초 만에** 잡았습니다.

**한 번 이상이 되면 자동 복구하지 않습니다.** LIO 는 한번 틀어지면 스스로
돌아오지 않기 때문입니다. 파이프라인을 재시작해야 합니다.

### B-4. `tools/lio_tf.py` — TF 발행

`indoor_map → base_link` 변환을 냅니다. Nav2 costmap 을 쓰신다면 필요하고,
직접 만든 A* 라면 없어도 됩니다.

**신뢰도가 나빠지면 발행을 멈춥니다.** TF 조회 실패로 같은 사실을 알 수
있습니다.

### B-5. `tools/proximity_guard.py` — 근접 경고 (임시)

원시 점군으로 앞이 막혔는지만 판단해 `/indoor/safe` 를 냅니다.
**LIO 에 의존하지 않습니다** — 위치가 발산해도 정상 동작합니다.

회피 알고리즘이 아니라 **최소 안전장치**입니다. 팀원B 의 회피가 완성될
때까지의 임시 조치이며, 그 뒤에는 아래 계층의 이중 안전장치로 남기거나
빼시면 됩니다.

`run_indoor.sh` 에는 포함돼 있지 않습니다. 필요하시면 따로 띄우십시오.

```bash
python3 tools/proximity_guard.py
```

---

## C. 분석·검증 (효신 작업용, 참고만)

| 파일 | 용도 |
|---|---|
| `eval_lio.py`, `summarize.py` | LIO 궤적 평가·집계 |
| `dump_odom.py` | bag → CSV |
| `run_lio.sh` | 실험 1회 자동 수행 (알고리즘 비교용) |
| `spin_check.py` | 제자리 회전으로 외부 파라미터 검증 |
| `compare_lio_gps.py` | LIO 두 개를 GPS 기준으로 비교 |
| `legodom_vs_gps.py` | 다리 오도메트리 표류 측정 |
| `imu_deadreckon.py` | IMU 단독 추측항법 |
| `gnss_bridge.py` | `/gnss` JSON → `NavSatFix` (**실외용**) |
| `gnss_path.py` | GPS 궤적 시각화 (**실외용**) |
| `gnss_dropout_probe.py` | GPS 끊김 원인 추적 |
| `plot_legodom_gps.py` | 표류 그림 |
| `go2_calib.py` | 외부 파라미터 상수 (다른 도구들이 import) |
| `lever_check.py`, `rec_rviz.sh` | 검증·영상 녹화 |

### C-1. 재현성·루프클로저·지도 검증 (2026-08-07 추가)

Point-LIO 재현성 문제를 추적하다 다리 오도메트리 전환으로 이어진 과정에서
만든 도구입니다. 배경·판정 근거는 `docs/2026-08-07_실험기록.md` 참고.

| 파일 | 용도 |
|---|---|
| `repro_run.sh` | 재현성 실험 1회 자동 실행 (T2→T3 간격 고정, 사람 손 제거) |
| `repro_all.sh` | 동일 조건 N회 반복 (`repro_run.sh` + 쿨다운) |
| `repro_monitor.py` | 실행 중 처리 프레임·궤적·CPU 관측 |
| `repro_report.py` | 반복 실행 결과를 한 표로 집계 (3층 판정: 브리지→LIO 처리량→지도) |
| `repro_diverge.py` | 몇 회차부터·어디서 갈라지는지 강체 정합(Kabsch)으로 추적 |
| `repro_yaw.py` | 헤딩 차이가 언제 벌어지는지 시계열로 추적 |
| `repro_event.py` | 발산 순간의 속도·회전율·위치 |
| `legodom_check.py` | bag 의 다리 오도메트리 드리프트 측정 (시작-끝 거리 ÷ 경로 길이) |
| `odom_map_build.py` | 다리 오도메트리 자세로 점군 누적 (A-4 참고) |
| `loop_correct_v2.py` | 루프 클로저 v2·증분 재적분, 현재 채택 (A-4 참고) |
| `loop_correct.py` | 루프 클로저 v1·전역 나선. **결함으로 폐기**, 참고용 보관 |
| `loop_correct_manual.py` | 실측(줄자) 제약을 직접 입력하는 v1 기반 변형 |
| `map_split_check.py` | 왕복 구간이 겹쳐 그려졌는지(드리프트 여부) 판정 |
| `grid_compare.py` | 무보정/보정 격자를 나란히 + 겹쳐서 비교 |
| `height_band_compare.py` | 높이 필터 범위 5종을 한 번에 비교 |
| `ground_inspect.py` | 어디를 바닥으로 잡고 있는지 시각화 (칸별 바닥 높이) |
| `pillar_inspect.py` | 기둥 하나를 확대해 정합 뭉개짐을 정량화 |
| `roi_time_inspect.py` | 구역을 관측 시각으로 색칠해 국소 드리프트를 드러냄 (전역 지표가 가리는 오차용) |
| `lidar_timing.py` | L1 발행 주기·프레임당 점 개수·회전 중 번짐 계산 |

---

## D. 받으실 토픽

### 필수

```
/indoor/base_pose      nav_msgs/Odometry       약 15 Hz
```

| 필드 | 내용 |
|---|---|
| `pose.pose.position` | 위치 x, y, z [m] |
| `pose.pose.orientation` | 자세 (쿼터니언) |
| `twist.twist` | 속도 |
| **`pose.covariance[0]`** | **신뢰도. 100 넘으면 정지** |

QoS: `rclcpp::SensorDataQoS()`

### 선택

```
/indoor/map            nav_msgs/OccupancyGrid   1회 latch
/cloud_registered      sensor_msgs/PointCloud2  실시간 정합 점군 (팀원B)
/tf                    indoor_map -> base_link
/indoor/health         std_msgs/Bool            진단용
/indoor/health_info    std_msgs/String          사유 JSON
/indoor/safe           std_msgs/Bool            근접 경고 (임시)
```

`/indoor/map` QoS: `rclcpp::QoS(1).transient_local().reliable()`

---

## D-2. 메시지 구조와 필드 이름

메시지 정의(`.msg`)가 C++ 구조체로 자동 생성되므로 **정해진 이름만** 쓸 수
있습니다. 없는 이름을 쓰면 컴파일 오류가 납니다.

직접 확인하시려면:

```bash
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show nav_msgs/msg/OccupancyGrid
ros2 topic echo /indoor/base_pose --once      # 실제 값
```

### `/indoor/base_pose` — `nav_msgs/Odometry`

```
Odometry
├── header
│   ├── stamp                      시각
│   └── frame_id      string       "indoor_map"
├── child_frame_id    string       "base_link"
├── pose                           ← PoseWithCovariance
│   ├── pose                       ← Pose
│   │   ├── position
│   │   │   ├── x     double       [m]
│   │   │   ├── y     double       [m]
│   │   │   └── z     double       [m]
│   │   └── orientation
│   │       └── x, y, z, w  double  쿼터니언
│   └── covariance    double[36]   ← [0] 이 신뢰도
└── twist                          ← TwistWithCovariance
    ├── twist
    │   ├── linear    x, y, z      [m/s]
    │   └── angular   x, y, z      [rad/s]
    └── covariance    double[36]
```

**`pose.pose` 가 두 번 나옵니다.** 바깥은 `PoseWithCovariance`(자세 + 공분산),
안쪽이 실제 `Pose` 입니다. 헷갈리기 쉬운 부분입니다.

```cpp
void onPose(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;      // ✓
  double y = msg->pose.pose.position.y;      // ✓
  double c = msg->pose.covariance[0];        // ✓  안쪽 pose 아님

  // int x = msg->pose.pose.position.x;      // ✗ 소수점이 잘린다
  // msg->pose.position.x                    // ✗ pose 하나 빠짐
  // msg->position.x                         // ✗ 없는 필드
}
```

- `SharedPtr` 이므로 **첫 접근만 `->`**, 그 뒤는 전부 `.` 입니다
- 좌표는 **`double`** 입니다. `int` 로 받으면 10 cm 격자에서 위치가 뭉개집니다

헤딩은 쿼터니언에서 뽑습니다.

```cpp
const auto & q = msg->pose.pose.orientation;
double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
```

### `/indoor/map` — `nav_msgs/OccupancyGrid`

```
OccupancyGrid
├── header
│   └── frame_id      string       "indoor_map"
├── info                           ← 격자의 기하 정보가 전부 여기
│   ├── resolution    double       0.10      한 칸 크기 [m]
│   ├── width         uint32       370       가로 칸 수
│   ├── height        uint32       332       세로 칸 수
│   └── origin                     ← 지도 원점
│       ├── position
│       │   ├── x     double       -22.2443
│       │   ├── y     double        -6.3021
│       │   └── z     double         0.0
│       └── orientation  (w=1, 회전 없음)
└── data              int8[]       격자 값. 0 자유 / 100 장애물 / -1 미지
```

```cpp
void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  double res = msg->info.resolution;
  double ox  = msg->info.origin.position.x;
  double oy  = msg->info.origin.position.y;
  int    w   = static_cast<int>(msg->info.width);
  int    h   = static_cast<int>(msg->info.height);

  // data 는 1차원. 2차원으로 보려면 직접 계산한다
  //   data[row * width + col]
  // row 0 이 아래쪽 (.pgm 과 반대)
}
```

### 좌표 변환

**로봇 위치는 이미 지도 좌표입니다.** 별도 변환이 필요 없고, 격자 칸 번호로
바꿀 때만 계산합니다.

```cpp
int col = static_cast<int>((x - ox) / res);
int row = static_cast<int>((y - oy) / res);
```

예: `x = 0.0`, `ox = -22.2443`, `res = 0.10` → `col = 222`

되돌릴 때는 칸의 **중심**을 씁니다. 격자 (3,5) 는 한 점이 아니라 10×10 cm
영역이므로, 모서리가 아니라 중심을 목표로 삼아야 로봇이 칸 가장자리를
스치지 않습니다.

```cpp
double x = ox + (col + 0.5) * res;
double y = oy + (row + 0.5) * res;
```

`static_cast<int>` 는 내림입니다. 격자 칸 번호이므로 이게 맞습니다. 반올림하면
칸 경계에서 한 칸씩 밀립니다.

### `origin` 이 왜 음수인가

격자 **왼쪽 아래 구석의 실좌표**입니다. 로봇이 시작점에서 사방으로 돌아다녔기
때문에, 서쪽 22 m·남쪽 6 m 지점이 격자 (0,0) 칸이 됩니다. 시작점은 격자
한가운데쯤에 있습니다.

**지도를 새로 만들면 이 값이 바뀝니다.** 코드에 상수로 박지 마시고 yaml 이나
토픽에서 읽으십시오.

---

## E. 지도 파일

```
results/indoor_map_inflated.pgm    ← A* 는 이것
results/indoor_map_inflated.yaml
results/indoor_map_inflated.npy    numpy 판
results/indoor_map.*               부풀리지 않은 원본 (표시용)
results/indoor_map_preview.png     어떻게 생겼는지
```

`_inflated` 는 장애물을 **25 cm 부풀린** 것입니다. 로봇 폭 여유이며, A* 는
이쪽을 써야 경로가 벽에 붙지 않습니다. 부풀린 뒤에도 자유공간이 100% 하나로
연결돼 통로가 막히지 않는 것을 확인했습니다.

값 규약이 셋 다 다릅니다.

| | 장애물 | 자유 | y축 |
|---|---|---|---|
| `.pgm` | 0 (검정) | 254 | **뒤집힘** |
| `.npy` | 1 | 0 | 안 뒤집힘 |
| `OccupancyGrid` | 100 | 0 | 안 뒤집힘 |

좌표 변환은 셋 다 같습니다.

```
col = (x - origin_x) / resolution
row = (y - origin_y) / resolution
```

---

## F. 알아 두셔야 할 것

**Point-LIO 는 회차마다 결과가 갈립니다.** 같은 녹화본을 여러 번 돌려도
성공할 때와 실패할 때가 있습니다. 원인 미규명이며 `covariance` 검사가
현재로선 유일한 방어선입니다.

| 환경 | 시작점 복귀 오차 (참값 0.19 m) |
|---|---|
| 실내 방 | 0.79 m |
| 복도 (성공) | 0.24 ± 0.01 m |
| 복도 (실패) | 18 ~ 52 m |
| 실외 운동장 | 110 m ~ 27 km |

> **2026-08-07 갱신**: 위 표는 Point-LIO 실시간 위치추정 결과입니다.
> **실내 지도 생성은 더 이상 이 경로를 쓰지 않습니다** — 다리 오도메트리
> 점군 누적으로 전환했습니다(같은 bag 기준 드리프트 2.63~2.90%, Point-LIO
> 는 21.75%). Point-LIO 는 원인 미규명 상태로 **보류**이며, 상세 비교와
> 정정된 지표(축 정렬 범위는 폐기, 회전 불변 지표 사용)는
> `docs/2026-08-07_실험기록.md` 참고.

**실외에서는 이 파이프라인을 쓰지 마십시오.** 지면 평면밖에 안 보여 방향이
제약되지 않습니다. 실외는 GPS 가 주 위치원이며 별도 파이프라인입니다.

**세션이 바뀌면 원점도 바뀝니다.** 좌표계 원점은 LIO 를 켠 순간 라이다가
있던 자리입니다. 저장된 지도를 다음 날 쓰려면 재위치추정이 필요한데 아직
구현돼 있지 않습니다. 지금은 **"켜고 → 지도 만들고 → 그 세션에서 주행"** 입니다.

**연결은 USB-이더넷을 쓰십시오.** WiFi 는 `/lowstate` 와 `/utlidar/imu` 가
밀립니다(실측: 500 → 171 Hz, 250 → 105 Hz). 내장 랜포트는 주행 중 끊긴
사례가 있습니다.

**주행 시험 중에는 반드시 리모컨을 들고 계십시오.**

---

## G. 문제 해결

| 증상 | 확인 |
|---|---|
| 토픽 미수신 | `./tools/doctor.sh robot`. USB-이더넷 연결 |
| 구독했는데 콜백이 안 불림 | **QoS 불일치.** 지도는 `transient_local`, 위치는 `SensorDataQoS` |
| `health=false` 가 자주 뜸 | LIO 발산. 재시작. 자동 복구 안 됨 |
| 지도 z 범위가 큼 | 이전 PCD 가 섞임. 파일 지우고 다시 |
| 지도 벽이 끊겨 보임 | 격자 해상도를 0.10 으로 |
| 회전할 때 위치가 튐 | `robot_pose` 가 떠 있는지 (LEVER 보정) |
| PCD 가 저장 안 됨 | `Ctrl+C` 로 정상 종료해야 함. 강제 종료 금지 |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | 스크립트 최신본으로 `git pull` |
| `loop_correct_v2.py` 보정 후 지도가 이상함 | 로그의 `fitness` 확인. **0.9 미만이면 보정하지 말 것** — 무보정본 사용 |

---

## H. `external/` — 외부 패키지 수정본 백업

`go2_ws`(URDF)와 `catkin_point_lio_unilidar`(Point-LIO)는 git 저장소가
아니라서 이 저장소 밖에 있으면 백업이 전혀 없었습니다. 소실 시 복구
불가였기 때문에 2026-08-08 에 이 저장소로 옮겼습니다.

| 폴더 | 원본 | 실제로 바꾼 것 |
|---|---|---|
| `point_lio_config/` | `dfloreaa/point_lio_ros2` | 설정 4줄 — 토픽명, `l1_imu_fixed` 연결, 다운샘플 옵션, `odom_child_frame_id` |
| `unitree_setup/` | `unitreerobotics/unitree_ros2` | `setup_go2.sh` 신규 작성 (NIC 자동 감지 + CycloneDDS) |
| `go2_description/` | 유니트리 공식 URDF (ROS1) | ROS2 `ament_cmake` 로 변환 |

복원 절차·diff 전문·`.bak` 이력은 `external/README.md` 를 보십시오.

**Point-LIO 설정은 보류 상태지만 지우지 않고 그대로 보관합니다** — 21.75%
드리프트의 원인을 나중에 규명하거나, 실외 GPS 음영 구간용 LIO 로 다시 쓸
수 있기 때문입니다. `external/README.md` 에는 원인 규명 1순위 후보로
지목된 `gravity_align` 설정 이력도 정리돼 있습니다.
