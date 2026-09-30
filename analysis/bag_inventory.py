#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_inventory.py  --  ROS2 bag 디렉토리 자동 스캔 & 단계별 EKF bag 후보 추천

무엇을 하나:
  - 지정한 루트 디렉토리 아래의 모든 ROS2 bag(= metadata.yaml 을 가진 폴더)을 찾는다.
  - 각 bag의 metadata.yaml 을 '직접 파싱'해서 (ros2 실행/소싱 불필요, 매우 빠름):
      * duration
      * 아래 4개 대상 토픽의 존재 여부 + 메시지 수 + 기록 Hz
      * /l1_imu_fixed, /aft_mapped_to_init 를 재생성하는 데 필요한 raw 토픽 존재 여부
  - EKF 단계(A: IMU+leg / B: +GPS / C: +Point-LIO)별로
      * '직접 사용 가능'(토픽이 bag에 이미 있음)
      * '재생 필요'(raw 로부터 fix/bridge/LIO 노드를 함께 재생하면 가능)
    를 판정하고 후보를 추천한다.
  - 결과를 콘솔 표 + bag_inventory.csv + bag_inventory.md 로 저장한다.
    (bag_inventory.md 를 그대로 복사해서 다음 단계에 붙이면 됨)

주의:
  - 이 스크립트는 EKF를 만들지 않는다. inventory 전용.
  - inventory 는 '실내/실외', '움직임 품질', 'GPS fix 품질'을 알 수 없다.
    → GPS 단계(B/C) 후보는 반드시 사용자가 실외 주행 bag인지 직접 확인해야 한다.
    → 그래서 bag '이름'을 그대로 노출한다(사용자가 이름으로 판별하도록).

실행:
  python3 bag_inventory.py /경로/bag_루트            # 스캔할 최상위 폴더
  python3 bag_inventory.py /경로/bag_루트 --top 8    # 단계별 후보 상위 N개
  python3 bag_inventory.py .                          # 현재 폴더부터 스캔

의존성: PyYAML (ROS2 환경엔 기본 설치돼 있음). 없으면: pip install pyyaml
"""

import os
import sys
import csv
import argparse
from datetime import datetime

try:
    import yaml
except ImportError:
    sys.exit("[에러] PyYAML 이 필요합니다.  ->  pip install pyyaml  (또는 ROS2 소싱 후 실행)")

# ─────────────────────────────────────────────────────────────────────────────
# 토픽 정의 (필요하면 여기만 수정)
# ─────────────────────────────────────────────────────────────────────────────
T_IMU_FIXED = "/l1_imu_fixed"          # 대상: 수정 IMU (재생성 = /utlidar/imu + /lowstate)
T_LEG       = "/utlidar/robot_odom"    # 대상: leg odometry (로봇이 직접 발행, raw)
T_GNSS      = "/gnss"                   # 대상: GPS (raw; bridge 가 NavSatFix 로 변환)
T_LIO       = "/aft_mapped_to_init"    # 대상: Point-LIO pose (재생성 = /utlidar/cloud + 수정IMU)

# raw 재생성 재료
R_IMU_RAW   = "/utlidar/imu"           # L1 자이로
R_LOWSTATE  = "/lowstate"              # body IMU(가속) + tick
R_CLOUD     = "/utlidar/cloud"         # LiDAR 포인트클라우드 (Point-LIO 입력)

TARGET_TOPICS = [T_IMU_FIXED, T_LEG, T_GNSS, T_LIO]
RAW_TOPICS    = [R_IMU_RAW, R_LOWSTATE, R_CLOUD]


# ─────────────────────────────────────────────────────────────────────────────
def find_bags(root):
    """metadata.yaml 을 가진 디렉토리를 모두 찾는다 (= ROS2 bag)."""
    bags = []
    for dirpath, _dirnames, filenames in os.walk(root):
        if "metadata.yaml" in filenames:
            bags.append(dirpath)
    return sorted(bags)


def parse_bag(bag_dir):
    """metadata.yaml 파싱. 실패해도 죽지 않고 error 필드에 기록."""
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    rec = {
        "path": bag_dir,
        "name": os.path.basename(os.path.normpath(bag_dir)),
        "duration_s": None,
        "start_str": "",
        "topics": {},      # name -> count
        "error": "",
    }
    try:
        with open(meta_path, "r") as f:
            data = yaml.safe_load(f)
        info = (data or {}).get("rosbag2_bagfile_information")
        if not info:
            rec["error"] = "metadata.yaml 에 rosbag2_bagfile_information 없음"
            return rec

        dur_ns = ((info.get("duration") or {}).get("nanoseconds"))
        if dur_ns is not None:
            rec["duration_s"] = float(dur_ns) / 1e9

        start_ns = ((info.get("starting_time") or {}).get("nanoseconds_since_epoch"))
        if start_ns:
            try:
                rec["start_str"] = datetime.fromtimestamp(start_ns / 1e9).strftime("%Y-%m-%d %H:%M")
            except (OSError, ValueError):
                rec["start_str"] = ""

        for t in (info.get("topics_with_message_count") or []):
            tm = t.get("topic_metadata") or {}
            name = tm.get("name")
            if name:
                rec["topics"][name] = int(t.get("message_count", 0) or 0)
    except Exception as e:  # noqa: BLE001  (inventory 는 절대 죽으면 안 됨)
        rec["error"] = f"파싱 실패: {e}"
    return rec


def has(rec, topic):
    """토픽이 존재하고 메시지가 1개 이상인가."""
    return rec["topics"].get(topic, 0) > 0


def hz(rec, topic):
    c = rec["topics"].get(topic, 0)
    d = rec["duration_s"]
    if c and d and d > 0:
        return c / d
    return None


def stage_direct(rec):
    """bag 에 대상 토픽이 '직접' 들어있는 기준으로 도달 가능한 최고 단계."""
    a = has(rec, T_IMU_FIXED) and has(rec, T_LEG)
    b = a and has(rec, T_GNSS)
    c = b and has(rec, T_LIO)
    if c: return "C"
    if b: return "B"
    if a: return "A"
    return "-"


def stage_replay(rec):
    """raw 로부터 fix/bridge/LIO 를 재생하면 도달 가능한 최고 단계.
       /l1_imu_fixed 재생성 = /utlidar/imu + /lowstate 필요.
       leg odom 은 raw 자체.
       GPS 는 /gnss(raw) 를 bridge 로 변환.
       LIO 재생성 = /utlidar/cloud + (수정IMU 재생 가능) 필요."""
    imu_ok = has(rec, T_IMU_FIXED) or (has(rec, R_IMU_RAW) and has(rec, R_LOWSTATE))
    a = imu_ok and has(rec, T_LEG)
    b = a and (has(rec, T_GNSS))
    lio_ok = has(rec, T_LIO) or (has(rec, R_CLOUD) and imu_ok)
    c = b and lio_ok
    if c: return "C"
    if b: return "B"
    if a: return "A"
    return "-"


def stage_rank(s):
    return {"A": 1, "B": 2, "C": 3, "-": 0}[s]


def fmt_dur(d):
    if d is None:
        return "N/A"
    m, s = divmod(int(round(d)), 60)
    return f"{m:d}:{s:02d}"


def mark(rec, topic):
    if has(rec, topic):
        h = hz(rec, topic)
        return f"O({rec['topics'][topic]}" + (f"@{h:.0f}Hz)" if h else ")")
    return "x"


# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="ROS2 bag inventory & 단계별 EKF bag 후보 추천")
    ap.add_argument("root", nargs="?", default=".", help="스캔할 최상위 bag 디렉토리 (기본: 현재 폴더)")
    ap.add_argument("--top", type=int, default=8, help="단계별 후보 상위 N개 (기본 8)")
    ap.add_argument("--outdir", default=".", help="csv/md 저장 위치 (기본: 현재 폴더)")
    args = ap.parse_args()

    if not os.path.isdir(args.root):
        sys.exit(f"[에러] 폴더가 아님: {args.root}")

    bag_dirs = find_bags(args.root)
    if not bag_dirs:
        sys.exit(f"[결과] '{args.root}' 아래에서 ROS2 bag(metadata.yaml)을 찾지 못했습니다.")

    recs = [parse_bag(b) for b in bag_dirs]
    ok = [r for r in recs if not r["error"]]
    bad = [r for r in recs if r["error"]]

    for r in ok:
        r["direct"] = stage_direct(r)
        r["replay"] = stage_replay(r)

    # duration 긴 순 정렬 (None 은 뒤로)
    ok.sort(key=lambda r: (r["duration_s"] is None, -(r["duration_s"] or 0)))

    # ── 콘솔 표 ──────────────────────────────────────────────────────────────
    print("\n" + "=" * 100)
    print(f" ROS2 BAG INVENTORY   root={os.path.abspath(args.root)}")
    print(f" 발견 bag: {len(recs)}개  (정상 {len(ok)}, 오류 {len(bad)})")
    print("=" * 100)
    header = f"{'bag 이름':30} {'dur':>7} {'fixed':>10} {'leg':>10} {'gnss':>10} {'lio':>10} {'직접':>4} {'재생':>4}"
    print(header)
    print("-" * 100)
    for r in ok:
        nm = r["name"] if len(r["name"]) <= 30 else "…" + r["name"][-29:]
        print(f"{nm:30} {fmt_dur(r['duration_s']):>7} "
              f"{mark(r, T_IMU_FIXED):>10} {mark(r, T_LEG):>10} "
              f"{mark(r, T_GNSS):>10} {mark(r, T_LIO):>10} "
              f"{r['direct']:>4} {r['replay']:>4}")
    if bad:
        print("-" * 100)
        for r in bad:
            print(f"[오류] {r['name']}: {r['error']}")

    # ── 단계별 추천 ──────────────────────────────────────────────────────────
    def candidates(stage_letter):
        need = stage_rank(stage_letter)
        # 직접 가능 우선 → 그다음 재생 가능. 각 그룹 내 duration 긴 순.
        direct = [r for r in ok if stage_rank(r["direct"]) >= need]
        replay_only = [r for r in ok if stage_rank(r["replay"]) >= need
                       and stage_rank(r["direct"]) < need]
        direct.sort(key=lambda r: -(r["duration_s"] or 0))
        replay_only.sort(key=lambda r: -(r["duration_s"] or 0))
        return direct[:args.top], replay_only[:args.top]

    print("\n" + "=" * 100)
    print(" 단계별 bag 후보 추천")
    print(" (inventory 는 실내/실외·움직임·GPS품질을 모름 → GPS단계는 실외 bag인지 이름으로 직접 확인)")
    print("=" * 100)
    stage_desc = {
        "A": "Stage A : IMU + leg odom              (필요: /l1_imu_fixed, /utlidar/robot_odom)",
        "B": "Stage B : IMU + leg + GPS             (+ /gnss)  ← 실외 bag 이어야 의미있음",
        "C": "Stage C : IMU + leg + GPS + Point-LIO (+ /aft_mapped_to_init)  ← 실외 loop 권장",
    }
    replay_note = {
        "A": "재생 시 필요 노드: l1_imu_fix.py",
        "B": "재생 시 필요 노드: l1_imu_fix.py + gnss_bridge.py",
        "C": "재생 시 필요 노드: l1_imu_fix.py + gnss_bridge.py + Point-LIO(--start-offset 25)",
    }
    for st in ["A", "B", "C"]:
        d, ro = candidates(st)
        print(f"\n■ {stage_desc[st]}")
        if d:
            print("  [직접 사용 가능] (bag에 토픽이 이미 있음, 가장 권장)")
            for r in d:
                print(f"    - {r['name']:40} dur {fmt_dur(r['duration_s'])}  start {r['start_str']}")
        else:
            print("  [직접 사용 가능] 없음")
        if ro:
            print(f"  [재생 필요] ({replay_note[st]})")
            for r in ro:
                print(f"    - {r['name']:40} dur {fmt_dur(r['duration_s'])}  start {r['start_str']}")

    # ── CSV 저장 ─────────────────────────────────────────────────────────────
    csv_path = os.path.join(args.outdir, "bag_inventory.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["name", "path", "duration_s", "start",
                    "l1_imu_fixed_cnt", "robot_odom_cnt", "gnss_cnt", "aft_mapped_cnt",
                    "utlidar_imu_cnt", "lowstate_cnt", "utlidar_cloud_cnt",
                    "direct_stage", "replay_stage", "error"])
        for r in recs:
            w.writerow([
                r["name"], r["path"],
                f"{r['duration_s']:.1f}" if r["duration_s"] else "",
                r.get("start_str", ""),
                r["topics"].get(T_IMU_FIXED, 0), r["topics"].get(T_LEG, 0),
                r["topics"].get(T_GNSS, 0), r["topics"].get(T_LIO, 0),
                r["topics"].get(R_IMU_RAW, 0), r["topics"].get(R_LOWSTATE, 0),
                r["topics"].get(R_CLOUD, 0),
                r.get("direct", "-"), r.get("replay", "-"), r["error"],
            ])

    # ── Markdown 저장 (이걸 복사해서 다음 단계에 붙이면 됨) ────────────────────
    md_path = os.path.join(args.outdir, "bag_inventory.md")
    with open(md_path, "w") as f:
        f.write(f"# Bag Inventory  (root: {os.path.abspath(args.root)})\n\n")
        f.write(f"발견 bag {len(recs)}개 (정상 {len(ok)}, 오류 {len(bad)})\n\n")
        f.write("| bag 이름 | dur | fixed | leg | gnss | lio | 직접 | 재생 |\n")
        f.write("|---|---|---|---|---|---|---|---|\n")
        for r in ok:
            f.write(f"| {r['name']} | {fmt_dur(r['duration_s'])} "
                    f"| {mark(r, T_IMU_FIXED)} | {mark(r, T_LEG)} "
                    f"| {mark(r, T_GNSS)} | {mark(r, T_LIO)} "
                    f"| {r['direct']} | {r['replay']} |\n")
        if bad:
            f.write("\n**오류 bag**\n\n")
            for r in bad:
                f.write(f"- {r['name']}: {r['error']}\n")

    print("\n" + "=" * 100)
    print(f" 저장 완료:\n   {os.path.abspath(csv_path)}\n   {os.path.abspath(md_path)}")
    print(" → bag_inventory.md 내용을 복사해서 다음 단계에 붙여주세요.")
    print("=" * 100)


if __name__ == "__main__":
    main()
