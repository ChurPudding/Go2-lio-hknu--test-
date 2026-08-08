import gtsam
import numpy as np
import matplotlib
matplotlib.use('Agg')          # 창이 안 뜨는 환경 대비 (파일로 저장)
import matplotlib.pyplot as plt
from gtsam import Pose2, BetweenFactorPose2, PriorFactorPose2

# ── 1. 그래프와 초기값 컨테이너 ──────────────────────
graph = gtsam.NonlinearFactorGraph()
initial = gtsam.Values()

prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.02]))
odom_noise  = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.20, 0.20, 0.10]))
loop_noise  = gtsam.noiseModel.Diagonal.Sigmas(np.array([2.0, 2.0, 1.0]))

# ── 2. 시작점 고정 ──────────────────────────────────
graph.add(PriorFactorPose2(0, Pose2(0, 0, 0), prior_noise))

# ── 3. 정사각형 주행 (회전이 매번 5도 부족하다고 가정) ──
drift_turn = np.deg2rad(85)

pose = Pose2(0, 0, 0)
initial.insert(0, pose)

for i in range(8):
    if i % 2 == 0:
        meas = Pose2(2.0, 0, 0)          # 전진 2m
    else:
        meas = Pose2(0, 0, drift_turn)   # 회전 (오차 포함)

    graph.add(BetweenFactorPose2(i, i + 1, meas, odom_noise))
    pose = pose.compose(meas)
    initial.insert(i + 1, pose)

# ── 4. 루프클로저 없이 최적화 ────────────────────────
result_before = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()

# ── 5. 루프클로저 추가 후 재최적화 ───────────────────
graph.add(BetweenFactorPose2(8, 0, Pose2(0, 0, 0), loop_noise))
result_after = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()

# ── 6. 시각화 ───────────────────────────────────────
def extract(values, n=9):
    return np.array([[values.atPose2(i).x(), values.atPose2(i).y()]
                     for i in range(n)])

b, a = extract(result_before), extract(result_after)

plt.figure(figsize=(6, 6))
plt.plot(b[:, 0], b[:, 1], 'o--', label='before loop closure', color='tomato')
plt.plot(a[:, 0], a[:, 1], 'o-',  label='after loop closure',  color='steelblue')
plt.plot(0, 0, 'k*', markersize=15, label='start')
plt.axis('equal'); plt.grid(alpha=0.3); plt.legend()
plt.title('GTSAM Pose2 SLAM - loop closure effect')
plt.savefig('gtsam_loop.png', dpi=120)

print(f"루프클로저 전 종점 오차: {np.linalg.norm(b[8]):.3f} m")
print(f"루프클로저 후 종점 오차: {np.linalg.norm(a[8]):.3f} m")
print("saved: gtsam_loop.png")

# 각 노드가 얼마나 움직였는지 (오차 분산 확인용)
print("\n노드별 이동량 (m):")
for i in range(9):
    print(f"  node {i}: {np.linalg.norm(a[i] - b[i]):.3f}")
