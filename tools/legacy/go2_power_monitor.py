#!/usr/bin/env python3
import argparse, csv, os, time
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_go.msg import LowState
NUM_CELLS = 8


class PowerMonitor(Node):
    def __init__(self, outdir):
        super().__init__("go2_power_monitor")
        os.makedirs(outdir, exist_ok=True)
        stamp = datetime.now().strftime("%m%d_%H%M%S")
        self.csv_path = os.path.join(outdir, f"go2_power_log_{stamp}.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["t", "elapsed_s", "soc", "voltage_V", "current_A", "power_W", "energy_Wh"])
        self.energy_wh = 0.0
        self.power_sum = 0.0
        self.sample_count = 0
        self.last_time = None
        self.start_time = None
        self.last_soc = None
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.sub = self.create_subscription(LowState, "/lowstate", self.callback, qos)
        self.last_print = 0.0
        self.get_logger().info(f"소비전력 기록 시작 -> {self.csv_path}")

    def callback(self, msg):
        now = time.time()
        bms = msg.bms_state
        voltage = sum(bms.cell_vol[:NUM_CELLS]) / 1000.0
        current = abs(bms.current) / 1000.0
        soc = bms.soc
        power = voltage * current
        if self.last_time is not None:
            self.energy_wh += power * ((now - self.last_time) / 3600.0)
        else:
            self.start_time = now
        self.last_time = now
        self.power_sum += power
        self.sample_count += 1
        self.last_soc = soc
        elapsed = now - self.start_time if self.start_time else 0.0
        self.writer.writerow([f"{now:.3f}", f"{elapsed:.2f}", soc, f"{voltage:.3f}", f"{current:.3f}", f"{power:.2f}", f"{self.energy_wh:.4f}"])
        if now - self.last_print >= 1.0:
            self.last_print = now
            avg = self.power_sum / self.sample_count
            print(f"[{elapsed:6.1f}s] SOC {soc}% | {voltage:.2f}V | {current:.2f}A | {power:.1f}W | 누적 {self.energy_wh:.2f}Wh | 평균 {avg:.1f}W")

    def print_summary(self):
        if self.sample_count == 0:
            return
        elapsed = (self.last_time - self.start_time) if self.start_time else 0.0
        avg_power = self.power_sum / self.sample_count
        print("\n" + "=" * 50)
        print("소비전력 측정 요약")
        print("=" * 50)
        print(f"  측정 시간    : {elapsed:.1f} 초 ({elapsed/60:.1f} 분)")
        print(f"  평균 소비전력: {avg_power:.1f} W")
        print(f"  누적 전력량  : {self.energy_wh:.3f} Wh")
        print(f"  현재 잔량    : {self.last_soc}%")
        if avg_power > 0 and self.last_soc is not None:
            BATTERY_WH_FULL = 230.0
            remaining_wh = BATTERY_WH_FULL * (self.last_soc / 100.0)
            hours = remaining_wh / avg_power
            print(f"  예상 잔여시간: 약 {hours*60:.0f} 분 (평균 {avg_power:.0f}W, 배터리 {BATTERY_WH_FULL:.0f}Wh 가정)")
        print("=" * 50)
        print(f"  CSV 저장     : {self.csv_path}")

    def destroy_node(self):
        try:
            self.print_summary()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--outdir", default=os.path.expanduser("~"))
    args = p.parse_args()
    rclpy.init()
    node = PowerMonitor(outdir=args.outdir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
