# 데이터 목록 (2026-07-31)

## 유효 — 절대 삭제 금지

| 경로 | 용량 | 내용 |
|---|---|---|
| `go2_run_full/` | 331M | **모든 실내 실험의 기준 bag.** 110초, 32.5m 닫힌 루프 |
| `rosbag2_2026_07_30-16_50_28/` | 614M | 실외 15분. 라이다는 없고 `/gnss`·`/lf/sportmodestate` 만 유효 |
| `exp/` | 22M | FAST-LIO before/after/nofeat 각 3회 + Point-LIO |
| `videos/` | 11M | 시연 영상 A·B·C |
| `results/` | — | 다리 오도메트리 표류 그림 |

## 폐기 — 규약(srcoff + -r 0.5 + 3회) 도입 전 결과

판정 근거로만 남긴다. 새 실험에 쓰지 말 것.

| 경로 | 용량 |
|---|---|
| `_deprecated/plio_imufix2/` | 751M |
| `_deprecated/plio_after_1041/` | 86M |
| `_deprecated/plio_before_981/` | 82M |
| `_deprecated/out_0730_1302/` | 251M |
| `_deprecated/out_0730_1326/` | 746M |

### 추가 폐기 (2026-07-31 확인)

| 경로 | 용량 | 사유 |
|---|---|---|
| `_deprecated/go2_run1/` | 120M | 50초. 정지 구간·닫힌 루프 조건 미달 |
| `_deprecated/go2_run2/` | 240M | 107초인데 메시지 76k (full 은 136k). 토픽 유실 |
| `_deprecated/expA_*/` | 소량 | 실험 A(다운샘플) 판정 보류분 |
| `_deprecated/traj_*/` | 소량 | 초기 궤적 시험 |
| `_deprecated/*.csv` | 소량 | 폐기 규약 시절 결과 |

전체 `_deprecated/` 2.4 GB. `.gitignore` 로 저장소에서 제외.
디스크 여유 790 GB 이므로 삭제하지 않는다 — **규약 도입 전후 비교의 원자료**이며,
다른 각도로 재분석할 여지가 있다.
