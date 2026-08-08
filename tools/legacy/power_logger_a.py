#!/usr/bin/env python3
"""
방법 A - BMS 총 전력 로깅 노드 (baseline 측정용)

사용 예:
  # 1a 스탠드 락:  로봇을 StandUp 상태로 두고
  python3 power_logger_a.py --condition stand_lock --trial 1
  # 1b 균형 서기:  로봇을 BalanceStand 상태로 두고
  python3 power_logger_a.py --condition stand_balance --trial 1

- Ctrl+C 로 종료하면 요약(평균 W, 표준편차, 샘플수)을 출력.
- 상태 전환 직후 3~5초는 나중에 분석에서 버리세요(과도구간).
- 메시지 타입이 다르면(예: unitree_hg) import 한 줄만 바꾸면 됩니다.
"""
import argparse, csv, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_go.msg import LowState   # Go2: unitree_go/msg/LowState


class PowerLoggerA(Node):
    def __init__(self, condition, trial, out_path):
        super().__init__("power_logger_a")
        self.condition = condition
        self.trial = trial
        self.samples = []          # 전력(W) 누적
        self.t0 = time.time()
        self.last_print = self.t0

        self.f = open(out_path, "a", newline="", encoding="utf-8-sig")
        self.w = csv.writer(self.f)
        if self.f.tell() == 0:
            self.w.writerow(["timestamp(시각)", "condition(조건)", "trial(시행회차)",
                             "voltage_V(전압V)", "current_A(전류A)", "power_W(전력W)",
                             "soc(잔량%)", "cellsum_V(셀전압합V)", "motor_temp_max(모터최고온도C)"])

        # Unitree lowstate 는 보통 BEST_EFFORT 로 발행됩니다.
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.sub = self.create_subscription(
            LowState, "/lowstate", self.cb, qos)
        self.get_logger().info(
            f"[{condition} #{trial}] 로깅 시작. Ctrl+C 로 종료하세요.")

    def cb(self, msg):
        v = float(msg.power_v)
        i = abs(float(msg.bms_state.current)) / 1000.0   # mA -> A
        p = v * i
        soc = int(msg.bms_state.soc)
        cellsum = sum(c for c in msg.bms_state.cell_vol if c > 0) / 1000.0
        temps = [m.temperature for m in msg.motor_state if m.mode == 1]
        tmax = max(temps) if temps else 0

        self.samples.append(p)
        self.w.writerow([f"{time.time():.3f}", self.condition, self.trial,
                         f"{v:.3f}", f"{i:.4f}", f"{p:.2f}",
                         soc, f"{cellsum:.3f}", tmax])

        now = time.time()
        if now - self.last_print >= 1.0:          # 1초마다 현황 출력
            avg = sum(self.samples) / len(self.samples)
            self.get_logger().info(
                f"P={p:5.1f}W  누적평균={avg:5.1f}W  "
                f"soc={soc}%  Tmax={tmax}C  n={len(self.samples)}")
            self.last_print = now

    def summary(self):
        n = len(self.samples)
        if n == 0:
            print("샘플 없음. QoS/토픽/메시지 타입을 확인하세요.")
            return
        avg = sum(self.samples) / n
        std = math.sqrt(sum((x - avg) ** 2 for x in self.samples) / n)
        dur = time.time() - self.t0
        print("\n===== 요약 =====")
        print(f"조건       : {self.condition} #{self.trial}")
        print(f"샘플 수    : {n}   측정시간: {dur:.1f}s")
        print(f"평균 전력  : {avg:.2f} W")
        print(f"표준편차   : {std:.2f} W")
        print(f"에너지     : {avg * dur / 3600:.3f} Wh")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--condition", required=True,
                    help="stand_lock / stand_balance / march / ...")
    ap.add_argument("--trial", type=int, default=1)
    ap.add_argument("--out", default="power_log.csv")
    args = ap.parse_args()

    rclpy.init()
    node = PowerLoggerA(args.condition, args.trial, args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
