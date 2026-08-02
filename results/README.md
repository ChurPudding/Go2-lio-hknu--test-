# results

## A* 에 쓸 지도

    indoor_map_inflated.pgm / .yaml / .npy      ← 이것을 쓰십시오

25 cm 부풀린 판입니다. 로봇 폭을 감안한 여유이며, 부풀린 뒤에도
자유공간이 100% 하나로 연결되어 통로가 막히지 않습니다.
사용법은 `docs/INTERFACE_indoor.md` 3절을 보십시오.

    indoor_map.*            부풀리지 않은 원본. 화면 표시용
    indoor_map_preview.png  지도가 어떻게 생겼는지

## 그 외

`corridor_map*` 은 같은 복도의 이전 판입니다. 참고용으로만 두었습니다.
`*_vs_gps.png`, `outdoor_*` 는 실외 LIO 평가 결과 그림이며
실내 주행과는 무관합니다.
