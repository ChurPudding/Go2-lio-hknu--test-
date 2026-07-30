# 런북 01 — 오프라인 검증 (bag 재생)

목적: `/l1_imu_fixed` 적용 후 남은 두 과제를 해결한다.
1. 수평 경로 44% 과대 추정의 원인 확인 (`publish_odometry_without_downsample`)
2. FAST-LIO도 같은 IMU 문제를 겪는지 확인 → 세 알고리즘 비교의 출발선 맞추기

소요 시간: 실험 하나당 약 4분 (재생 110초 + 준비). 전체 30~40분.

---

## 0. 공통 준비

**모든 터미널에서 매번** 아래 두 줄을 먼저 실행한다. `unitree_go` 메시지와 `point_lio` 둘 다 필요하다.

```bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/catkin_point_lio_unilidar/install/setup.bash
```

편의를 위해 별칭을 만들어 두면 좋다.

```bash
echo "alias srclio='source ~/unitree_ros2/cyclonedds_ws/install/setup.bash && source ~/catkin_point_lio_unilidar/install/setup.bash'" >> ~/.bashrc
source ~/.bashrc
```

### 0.1 브리지 노드 행렬 갱신 확인

정밀화된 `R_LB`이 반영됐는지 본다.

```bash
grep -n "0.523029" ~/fastlio_ws/tools/l1_imu_fix.py
```

안 나오면 갱신본 `l1_imu_fix.py`로 교체한다. (기존 값과 0.294° 차이라 결과는 거의 같지만, 기록 일관성을 위해 맞춰둔다.)

### 0.2 현재 설정 스냅샷

```bash
SRC=~/catkin_point_lio_unilidar/src/point_lio_ros2/config/unilidar_l1.yaml
grep -nE "imu_topic|acc_norm|gravity_align|gravity:|extrinsic|publish_odometry" "$SRC"
```

기대값:

| 항목 | 값 |
|---|---|
| `imu_topic` | `/l1_imu_fixed` |
| `acc_norm` | `9.81` |
| `gravity_align` | `true` |
| `gravity` / `gravity_init` | `[0.0, 0.0, -9.810]` |
| `extrinsic_R` | 단위행렬 |
| `extrinsic_T` | `[0.007698, 0.014655, -0.00667]` |

---

## 1. 실험 A — 출력 다운샘플 (최우선)

### 가설
수평 경로가 46.9 m(참값 32.5 m)로 44% 길게 나온 것은 `publish_odometry_without_downsample: true`로 인해 포인트 단위 8.5 kHz 출력이 나오면서 잡음이 그대로 경로 길이에 누적됐기 때문이다.

`R_LB`의 yaw는 −127.9° ± 0.5°로 확정되어 원인에서 제외됐으므로, 이것이 유일하게 남은 후보다.

### 설정

```bash
SRC=~/catkin_point_lio_unilidar/src/point_lio_ros2/config/unilidar_l1.yaml
cp "$SRC" "$SRC.bak_expA"
sed -i 's/^\( *publish_odometry_without_downsample:\) *true/\1 false/' "$SRC"
grep -n "publish_odometry_without_downsample" "$SRC"
```

### 실행

터미널 4개. 각각 `srclio` 먼저.

```bash
# [T1] 브리지 노드
python3 ~/fastlio_ws/tools/l1_imu_fix.py
```

```bash
# [T2] Point-LIO — "incompatible QoS" 경고가 없어야 함
ros2 launch point_lio mapping_unilidar_l1.launch.py
```

```bash
# [T3] 녹화 — "Recording..." 확인 후 잠깐 대기
ros2 bag record -o ~/fastlio_ws/expA_downsample /aft_mapped_to_init
```

```bash
# [T4] 재생 — 마지막에 시작
cd ~/fastlio_ws && ros2 bag play go2_run_full
```

재생 완료 후 종료 순서: **T4 → T3 → T2 → T1**

### 확인 및 추출

```bash
ros2 bag info ~/fastlio_ws/expA_downsample
```

`Count`가 0이 아니어야 한다. 이번엔 다운샘플이 켜지므로 이전(93만)보다 훨씬 적게(수천~수만) 나오는 것이 정상이다.

```bash
python3 ~/fastlio_ws/dump_odom.py \
  ~/fastlio_ws/expA_downsample/expA_downsample_0.db3 \
  ~/fastlio_ws/expA.csv
wc -l ~/fastlio_ws/expA.csv
```

### 판정 기준

| 지표 | 참값 | 직전 결과 | 목표 |
|---|---|---|---|
| 수평 총거리 | 32.5 m | 46.9 m | 35 m 이하 |
| 루프 클로저 | 0.56 m | 1.21 m | 0.8 m 이하 |
| z RMSE | — | 25.6 mm | 유지 또는 개선 |
| 최종 z 오차 | +1.8 mm | −1.4 mm | 유지 |

수평이 개선되고 z가 나빠지지 않으면 `false`를 기본값으로 확정한다.

---

## 2. 실험 B — FAST-LIO에 동일 수정 적용

### 가설
FAST-LIO도 같은 `/utlidar/imu`를 쓰고 있으므로 동일한 가속도계 문제를 안고 있다. 다만 알고리즘 구조가 달라(FAST-LIO는 스캔 단위, Point-LIO는 포인트 단위) 증상이 완전 발산이 아니라 완만한 열화로 나타났을 가능성이 있다.

### 설정

FAST-LIO config(`go2_l1.yaml`)에서 다음을 바꾼다.

```yaml
common:
    imu_topic: "/l1_imu_fixed"        # was /utlidar/imu
```

**주의:** FAST-LIO는 `extrinsic_est_en: false`, `extrinsic_R` 단위행렬, `timestamp_unit: 0`을 그대로 유지한다. 이 값들은 이미 검증됐고, IMU 프레임 = LiDAR 프레임 관계가 그대로 유지되므로 바꿀 이유가 없다.

FAST-LIO에 `gravity_align` 계열 파라미터가 있으면 Point-LIO와 동일하게 맞춘다.

### 실행

실험 A와 동일한 4터미널 구조. FAST-LIO의 오도메트리 토픽명을 먼저 확인한다.

```bash
ros2 topic list | grep -iE "aft_mapped|Odometry|path"
```

```bash
ros2 bag record -o ~/fastlio_ws/expB_fastlio_fixed <토픽명>
```

**대조군도 필요하다.** `imu_topic`을 `/utlidar/imu`로 되돌려 한 번 더 녹화한다(`expB_fastlio_orig`). 같은 bag, 같은 조건으로 두 개가 있어야 정량 비교가 된다.

### 판정 기준

`/l1_imu_fixed` 쪽이 z RMSE와 루프 클로저 모두에서 개선되면, **세 알고리즘 비교 실험 전체를 `/l1_imu_fixed` 기준으로 다시 돌려야 한다**는 결론이 된다. 지금까지의 FAST-LIO vs Point-LIO 비교는 IMU가 깨진 상태에서 이뤄진 것이므로 무효다.

---

## 3. 실험 C — `/body_imu` 단독 (선택)

### 가설
`R_BL`이 확정됐으므로 하이브리드 대신 본체 IMU만 써도 된다. 500 Hz라 대역이 넓고, 자이로 적분 오차가 110초에 2.6%로 검증됐다.

### 설정

```yaml
common:
    imu_topic: "/body_imu"            # body_imu_bridge.py 출력
mapping:
    acc_norm: 9.465                   # 본체 가속도계 실측 정지 |a|
    gravity_align: true
    gravity:      [0.0, 0.0, -9.465]
    gravity_init: [0.0, 0.0, -9.465]
    imu_time_inte: 0.002              # 500 Hz
    extrinsic_T: [0.0, 0.0, 0.0]      # 별도 측정 필요, 우선 0
    extrinsic_R: [ +0.523029, -0.810712, +0.263034,
                   -0.838576, -0.544668, -0.011292,
                   +0.152420, -0.214668, -0.964721 ]
```

`extrinsic_R`은 **LiDAR → 본체(IMU) 회전**, 즉 `R_BL`이다. Point-LIO의 extrinsic 정의가 "IMU body frame에서 본 LiDAR의 자세"이므로 이 방향이 맞다.

`body_imu_bridge.py`가 가속도 스케일 보정을 하지 않는다면 `acc_norm: 9.465`, 9.807로 정규화한다면 `acc_norm: 9.81`을 쓴다. **둘 중 어느 쪽인지 코드에서 확인할 것.**

### 판정
실험 A 결과와 직접 비교한다. 하이브리드보다 나으면 이쪽으로 갈아탄다.

---

## 4. 결과 기록 양식

```
실험: ___________  일시: ___________
설정 변경: ___________________________

수평 총거리 ______ m   (참값 32.5)
루프 클로저  ______ m   (참값 0.56)
z RMSE      ______ mm
z 최대오차   ______ mm
최종 z 오차  ______ mm  (참값 +1.8)

회전 구간 z 변화 (참값 / 결과 / 차이, mm)
  R2  (15.7~18.5s)  +4.0 / ____ / ____
  R3  (27.7~30.5s)  -6.1 / ____ / ____
  R6  (38.2~40.7s)  -4.1 / ____ / ____
  R8  (46.5~49.3s)  +3.7 / ____ / ____
  R9  (51.3~53.7s)  +1.6 / ____ / ____
  R14 (85.5~89.2s)  -2.8 / ____ / ____

RViz 육안: 벽 정렬 (선명/번짐), 궤적 폐합 (양호/불량)
메모:
```

---

## 5. 문제 발생 시

| 증상 | 원인 | 조치 |
|---|---|---|
| `Package 'point_lio' not found` | source 누락 | `srclio` 실행 |
| `incompatible QoS ... RELIABILITY` | 브리지 퍼블리셔가 BEST_EFFORT | `l1_imu_fix.py`의 `pub_qos.reliability = ReliabilityPolicy.RELIABLE` 확인 |
| `ros2 bag info` Count = 0 | record가 재생보다 늦게 시작 | record → 재생 순서 엄수, "Recording..." 확인 후 재생 |
| `/l1_imu_fixed` 안 나옴 | `/lowstate` 미수신 | `ros2 topic hz /lowstate` 확인. bag 재생 중이어야 함 |
| `ModuleNotFoundError: unitree_go` | unitree 워크스페이스 미source | `srclio` 실행 |
| 궤적이 다시 발산 | IMU 토픽이 되돌아감 | yaml `imu_topic` 확인, 노드 실행 여부 확인 |

### 정지 구간 IMU 값 확인 (문제 시 최우선 점검)

```bash
# echo를 먼저 띄우고 나서 bag 재생
ros2 topic echo /l1_imu_fixed --field linear_acceleration > /tmp/acc.txt
# 5초 후 Ctrl+C
head -20 /tmp/acc.txt
```

기대값 **(x ≈ +1.6, y ≈ −1.8, z ≈ −9.5)**. z가 양수면 회전 행렬이 적용되지 않은 것이다.

---

## 6. 다음 단계

실험 A·B가 끝나면 세 알고리즘(FAST-LIO / FAST-LIO2 / Point-LIO)을 모두 `/l1_imu_fixed` 기준으로 재평가한다. 이때 비교 지표는 이 문서와 동일하게 쓴다.

- z RMSE, 최종 z 오차 → `robot_odom` z 기준 (참값 ±16 mm)
- 루프 클로저 → 시작-끝 수평거리 (참값 0.56 m)
- 수평 총거리 → 참값 32.5 m
- 실시간성 → 처리 시간, 드롭 프레임 수
