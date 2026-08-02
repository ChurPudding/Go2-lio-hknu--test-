# 실내 위치추정 인터페이스 (팀원A용)

담당: 효신 (LIO 위치추정·지도)
갱신: 2026-08-02

---

## 1. 받으실 것은 토픽 하나입니다

```
/indoor/base_pose        nav_msgs/Odometry        약 15 Hz
```

| 필드 | 내용 |
|---|---|
| `header.stamp` | 로봇 시각 |
| `header.frame_id` | `camera_init` (= 지도 원점) |
| `pose.pose.position` | 몸통 중심 위치 x, y, z [m] |
| `pose.pose.orientation` | 자세 (쿼터니언) |
| `twist.twist` | 속도 |
| **`pose.covariance[0]`** | **위치 신뢰도** |

## 2. 반드시 넣어 주셔야 할 안전 조건

```python
def on_pose(self, msg):
    if msg.pose.covariance[0] > 100:
        self.stop()          # 위치를 믿을 수 없음 -> 즉시 정지
        return

    x   = msg.pose.pose.position.x
    y   = msg.pose.pose.position.y
    yaw = 2 * math.atan2(msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w)
    ...
```

| covariance[0] | 뜻 |
|---|---|
| 0.01 | 정상 |
| 1000000.0 | **위치 추정 실패. 이 좌표를 쓰면 안 됨** |

### 왜 필요한가

LIO 는 실패해도 조용히 틀린 좌표를 계속 내보냅니다. 실제로 복도 실험에서
7×6 m 를 도는데 출발점에서 **52 m 벗어난** 회차가 있었습니다. 이 조건이 없으면
로봇은 자신 있게 벽으로 갑니다.

감시 노드가 네 가지를 봅니다 — 수신 끊김, 정지 중 표류, 로봇 자체 속도와의
불일치, 높이 급변. 복도 실패 데이터에서 **12.9초 만에** 잡았습니다.

**한 번 이상이 되면 자동으로 회복하지 않습니다.** LIO 는 한번 틀어지면 스스로
돌아오지 않기 때문입니다. 파이프라인을 다시 시작해야 합니다.

---

## 3. 지도

```
results/indoor_map_inflated.pgm     지도 이미지
results/indoor_map_inflated.yaml    해상도·원점
```

Nav2 표준 형식입니다. `.yaml` 내용:

```yaml
image: indoor_map_inflated.pgm
resolution: 0.1000
origin: [-22.2443, -6.3021, 0.0]
```

### 좌표 변환

`/indoor/base_pose` 의 좌표와 지도가 **같은 원점**을 씁니다.

```python
col = int((x - origin[0]) / resolution)
row = int((y - origin[1]) / resolution)
```

`.pgm` 은 위아래가 뒤집혀 저장되므로, 이미지로 읽으실 때는

```python
row_img = height - 1 - row
```

`.npy` 판(`indoor_map_inflated.npy`)을 쓰시면 뒤집을 필요가 없습니다.
`1 = 장애물`, `0 = 자유공간` 인 `uint8` 배열입니다.

### 부풀리기

`_inflated` 판은 장애물을 **25 cm 부풀린** 것입니다. 로봇 폭을 감안한 여유라
A* 는 이쪽을 쓰셔야 합니다. 부풀린 뒤에도 자유공간이 100% 하나로 연결되어
있으므로 통로가 막히지 않습니다.

부풀리지 않은 `indoor_map.*` 은 화면 표시용입니다.

---

## 4. 좌표계

지도와 위치는 **`camera_init`** 프레임입니다. LIO 를 켠 순간의 라이다 자세가
원점이며, **다시 켜면 원점이 바뀝니다.**

- 지도를 만든 세션과 주행 세션이 같으면 그대로 맞습니다
- 저장된 지도를 나중에 쓰시려면 재위치추정이 필요합니다 (현재 미구현)

TF 도 발행합니다. Nav2 costmap 을 쓰신다면 필요하고, 직접 만든 A* 라면 없어도
됩니다.

```
/tf     indoor_map -> base_link
```

`indoor_map` 은 `camera_init` 과 같은 원점입니다(항등 변환으로 묶어 둠).
**신뢰도가 이상해지면 TF 발행도 멈춥니다.** `lookupTransform` 이 실패하는 것으로
같은 사실을 알 수 있습니다.

---

## 5. 실행

```bash
cd ~/fastlio_ws
./tools/run_indoor.sh            # 실시간 (로봇 연결)
./tools/run_indoor.sh bag <경로> # 녹화본 재생
```

브리지 → Point-LIO → 위치 변환 → 감시 → TF 가 순서대로 뜹니다.
실시간 모드는 시작 전에 로봇 토픽 4개의 수신을 확인하고, 하나라도 없으면
중단합니다.

상태 확인:

```bash
tail -f /tmp/go2_indoor/health.log
```

---

## 6. 알아 두셔야 할 한계

**Point-LIO 는 회차마다 결과가 갈립니다.** 같은 녹화본을 여러 번 돌려도
성공할 때와 실패할 때가 있습니다. 원인 미규명이며, 그래서 위 안전 조건이
현재로서는 유일한 방어선입니다.

| 환경 | 루프 클로저 (참값 0.19 m) |
|---|---|
| 실내 방 | 0.79 m |
| 복도 (성공 회차) | 0.24 ± 0.01 m |
| 복도 (실패 회차) | 18 ~ 52 m |
| 실외 운동장 | 110 m ~ 27 km |

**실외에서는 이 인터페이스를 쓰지 마십시오.** 실외는 GPS 가 주 위치원이며
별도 파이프라인(`/gps/fix`)을 씁니다.

**주행 테스트 중에는 반드시 리모컨을 들고 계십시오.** 소프트웨어가 어디서
고장 나도 물리적으로 멈출 수 있는 건 그것뿐입니다.

---

## 7. 참고 — 진단용 토픽

필수는 아니지만 문제가 생겼을 때 쓰실 수 있습니다.

```
/indoor/health       std_msgs/Bool     false = 신뢰 불가
/indoor/health_info  std_msgs/String   사유 JSON
/cloud_registered    PointCloud2       실시간 정합 점군 (팀원B 용)
```
