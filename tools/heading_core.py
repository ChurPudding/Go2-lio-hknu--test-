#!/usr/bin/env python3
"""heading_core.py -- GPS 진행방향 기반 절대 방위 추정. ROS 비의존.

이 파일은 ROS 를 import 하지 않는다. 순수한 계산만 담는다.
- 실시간 노드(`gps_heading.py`)가 이것을 감싸서 토픽에 연결한다
- 검증 스크립트(`verify_heading.py`)가 bag 에서 직접 먹인다

같은 코드를 두 경로로 쓰므로, 오프라인에서 검증한 것이 실기에서도 같다.

원리
----
Go2 의 yaw 는 자이로 적분이라 (1) 전원 켠 방향이 0° 이고 (2) 2~5°/분 표류한다.
자기계가 없어 IMU 만으로는 해결할 수 없다.

로봇이 앞으로 걸으면 GPS 위치가 이동한 방향이 곧 몸통이 향한 방향이다.
단, 사족보행은 게걸음을 하므로 측면미끄러짐각 β 를 빼야 한다.

    몸통 방위 = GPS 진행방향 - β,     β = atan2(vy, vx)
    절대 방위 = IMU yaw + 오프셋

오프셋을 느리게 갱신한다. IMU 는 부드럽지만 표류하고, GPS 는 잡음이 크지만
표류하지 않는다. 서로를 보완한다.

파라미터 근거 (2026-08-02 실측)
------------------------------
기선 3 m — `tools/baseline_sweep.py`. 개별 잔차 9.5°, 지연 1.5 s.
짧게 잡아도 잡음이 이론(1/L)만큼 커지지 않는데, 수신기의 반송파 평활화로
연속 측위가 함께 움직여 차이에서 상쇄되기 때문이다. 1 m 는 5 m 대비
이론상 5배여야 하나 실측 1.4배였다.

시정수 60 s — `tools/verify_heading.py` 로 두 bag 교차 검증.

    tau     1114 편향/흔들림    1128 편향/흔들림
     30 s    +0.86 / 3.29        +0.37 / 3.15
     60 s    +1.06 / 1.95        +0.26 / 2.44
    120 s    +1.34 / 1.07        -0.92 / 1.24
    180 s    +1.46 / 0.92        -3.07 / 1.28

120 s 부터 두 bag 의 편향 부호가 갈린다. 표류를 못 따라가 생기는 지연이며,
데이터마다 방향이 다르다. 60 s 는 양쪽에서 안정적이고 성능도 충분하다
(반지름 20 m 에서 횡방향 0.8 m. GPS 측위 오차 2.4 m 보다 작다).

주의: 두 bag 모두 정지 구간이 짧아(15~34 s) 사각지대 복구를 평가하지
못했다. 긴 정지·회전이 포함된 실외 실험 후 재조정할 것.

사각지대
--------
정지·회전·게걸음 중에는 진행방향이 정의되지 않아 관측이 멈춘다.
그동안은 IMU 표류가 그대로 쌓인다. 30 초 정지에 1~2.5°.
"""
import math

R_EARTH = 6378137.0


def wrap(a):
    """각도를 [-pi, pi) 로 접는다."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class HeadingEstimator:
    """GPS + IMU 로 절대 방위를 추정한다.

    사용법
    ------
        est = HeadingEstimator()
        est.push_imu(t, yaw)             # 자주
        est.push_velocity(t, vx, vy)     # 자주
        est.push_gps(t, lat, lon)        # 1 Hz. 여기서 관측이 만들어진다
        h = est.heading()                # 절대 방위 [rad] 또는 None
    """

    def __init__(self,
                 baseline=3.0,          # m    기선 최소 길이
                 max_span=20.0,         # s    기선 최대 시간
                 bow_ratio=0.20,        #      직선 판정. 현 길이 대비 허용 편차
                 min_speed=0.20,        # m/s  이보다 느리면 관측 안 함
                 max_lateral=0.40,      # m/s  이보다 심한 게걸음은 버린다
                 beta_correct=True,     #      β 보정 사용
                 tau=60.0,              # s    오프셋 필터 시정수
                 init_tau=4.0,          # s    초기 수렴용 짧은 시정수
                 init_count=8,          #      이만큼 받을 때까지 init_tau
                 outlier_deg=40.0,      # deg  수렴 후 이보다 어긋나면 버린다
                 vel_window=3.0):       # s    속도 이력 보관 길이
        self.L = baseline
        self.max_span = max_span
        self.bow_ratio = bow_ratio
        self.min_speed = min_speed
        self.max_lat = max_lateral
        self.use_beta = beta_correct
        self.tau = tau
        self.init_tau = init_tau
        self.init_count = init_count
        self.outlier = math.radians(outlier_deg)
        self.vel_window = vel_window

        self.gps = []          # [(t, E, N)]
        self.yaws = []         # [(t, yaw)]  과거 시점 조회용
        self.vels = []         # [(t, vx, vy)]
        self.origin = None

        self.offset = None
        self.n_obs = 0
        self.n_reject = 0
        self.reject_reason = {'기선부족': 0, '곡선': 0, '저속': 0,
                              '게걸음': 0, '이상치': 0}
        self.last_obs_t = None
        self.last_gps_t = None
        self.last_resid = None
        self.log = []          # [(t, offset, resid)] 분석용

    # ------------------------------------------------------------------
    def push_imu(self, t, yaw):
        self.yaws.append((t, yaw))
        # 기선 최대 시간보다 오래된 것만 버린다 (과거 조회에 필요)
        cut = t - self.max_span - 5.0
        while len(self.yaws) > 2 and self.yaws[0][0] < cut:
            self.yaws.pop(0)

    def push_velocity(self, t, vx, vy):
        self.vels.append((t, vx, vy))
        cut = t - self.vel_window
        while len(self.vels) > 1 and self.vels[0][0] < cut:
            self.vels.pop(0)

    def push_gps(self, t, lat, lon):
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        self.last_gps_t = t
        if self.origin is None:
            self.origin = (math.radians(lat), math.radians(lon))
        lat0, lon0 = self.origin
        E = R_EARTH * (math.radians(lon) - lon0) * math.cos(lat0)
        N = R_EARTH * (math.radians(lat) - lat0)
        self.gps.append((t, E, N))
        while self.gps and t - self.gps[0][0] > self.max_span:
            self.gps.pop(0)
        self._try_observe()

    # ------------------------------------------------------------------
    def yaw_at(self, t):
        """t 시점의 yaw 를 선형 보간한다. 각도라 감싸기에 주의."""
        if not self.yaws:
            return None
        if t <= self.yaws[0][0]:
            return self.yaws[0][1]
        if t >= self.yaws[-1][0]:
            return self.yaws[-1][1]
        for k in range(len(self.yaws) - 1):
            t0, y0 = self.yaws[k]
            t1, y1 = self.yaws[k + 1]
            if t0 <= t <= t1:
                if t1 == t0:
                    return y0
                r = (t - t0) / (t1 - t0)
                return wrap(y0 + r * wrap(y1 - y0))
        return self.yaws[-1][1]

    def mean_velocity(self, t0, t1):
        sel = [(vx, vy) for (t, vx, vy) in self.vels if t0 <= t <= t1]
        if not sel:
            # 이력이 없으면 최신값으로 대신한다
            return (self.vels[-1][1], self.vels[-1][2]) if self.vels else None
        n = len(sel)
        return (sum(v[0] for v in sel) / n, sum(v[1] for v in sel) / n)

    # ------------------------------------------------------------------
    def _try_observe(self):
        if len(self.gps) < 2 or not self.yaws:
            return
        t1, E1, N1 = self.gps[-1]

        # 기선을 만족하는 가장 최근 과거 점 — 지연을 최소화한다
        idx = None
        for k in range(len(self.gps) - 2, -1, -1):
            t0, E0, N0 = self.gps[k]
            if math.hypot(E1 - E0, N1 - N0) >= self.L:
                idx = k
                break
        if idx is None:
            self.reject_reason['기선부족'] += 1
            return
        t0, E0, N0 = self.gps[idx]

        dE, dN = E1 - E0, N1 - N0
        chord = math.hypot(dE, dN)

        # 직선성 — 중간 점이 현에서 얼마나 벗어나는가
        for k in range(idx + 1, len(self.gps) - 1):
            _, Em, Nm = self.gps[k]
            if abs((Em - E0) * dN - (Nm - N0) * dE) / chord > \
                    self.bow_ratio * chord:
                self.reject_reason['곡선'] += 1
                self.n_reject += 1
                return

        mv = self.mean_velocity(t0, t1)
        if mv is None:
            return
        vx, vy = mv
        if vx < self.min_speed:
            self.reject_reason['저속'] += 1
            self.n_reject += 1
            return
        if abs(vy) > self.max_lat:
            self.reject_reason['게걸음'] += 1
            self.n_reject += 1
            return

        course = math.atan2(dN, dE)
        if self.use_beta:
            course -= math.atan2(vy, vx)

        # 이 관측이 유효한 시각은 기선의 중점이다
        tm = 0.5 * (t0 + t1)
        yaw_m = self.yaw_at(tm)
        if yaw_m is None:
            return

        self._update_offset(wrap(course - yaw_m), t1 - t0, t1)

    def _update_offset(self, obs, span, t):
        if self.offset is None:
            self.offset = obs
            self.n_obs = 1
            self.last_obs_t = t
            self.last_resid = 0.0
            self.log.append((t, self.offset, 0.0))
            return

        resid = wrap(obs - self.offset)
        self.last_resid = resid

        # 수렴 후에는 튀는 관측을 버린다 (회전 중 오관측 방지)
        if self.n_obs > self.init_count and abs(resid) > self.outlier:
            self.reject_reason['이상치'] += 1
            self.n_reject += 1
            return

        tau = self.init_tau if self.n_obs < self.init_count else self.tau
        alpha = 1.0 - math.exp(-max(span, 0.1) / tau)
        self.offset = wrap(self.offset + alpha * resid)
        self.n_obs += 1
        self.last_obs_t = t
        self.log.append((t, self.offset, resid))

    # ------------------------------------------------------------------
    def heading(self, t=None):
        """절대 방위 [rad]. ENU 기준(동쪽 0, 반시계 +). 아직 없으면 None."""
        if self.offset is None or not self.yaws:
            return None
        yaw = self.yaws[-1][1] if t is None else self.yaw_at(t)
        return wrap(yaw + self.offset)

    @property
    def converged(self):
        return self.n_obs > self.init_count

    def status(self, t):
        return {
            'heading_deg': (round(math.degrees(self.heading()), 2)
                            if self.heading() is not None else None),
            'offset_deg': (round(math.degrees(self.offset), 2)
                           if self.offset is not None else None),
            'n_obs': self.n_obs,
            'n_reject': self.n_reject,
            'reject': dict(self.reject_reason),
            'sec_since_obs': (round(t - self.last_obs_t, 1)
                              if self.last_obs_t else None),
            'sec_since_gps': (round(t - self.last_gps_t, 1)
                              if self.last_gps_t else None),
            'last_resid_deg': (round(math.degrees(self.last_resid), 2)
                               if self.last_resid is not None else None),
            'converged': self.converged,
        }
