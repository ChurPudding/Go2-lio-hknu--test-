#!/usr/bin/env python3
"""
fix_rosbag2_metadata.py

rosbags(0.10+) 또는 Jazzy 이상에서 생성된 rosbag2 metadata.yaml을
Humble(metadata version 5)이 읽을 수 있는 형태로 되돌린다.

증상:
    ros2 bag info <bag>
    -> Exception on parsing info file: yaml-cpp: error at line N, column M: bad conversion

원인:
    version 9에서 offered_qos_profiles가 문자열 -> YAML 시퀀스로 바뀌었고
    type_description_hash 필드가 추가됨. Humble의 yaml-cpp 파서가 시퀀스를
    문자열로 변환하려다 실패한다.

사용법:
    python3 fix_rosbag2_metadata.py ~/data/bags/exp/l1_official_ros2
    python3 fix_rosbag2_metadata.py ~/data/bags/exp/l1_official_ros2 --dry-run

원본은 metadata.yaml.bak 으로 백업된다.
"""

import argparse
import shutil
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("PyYAML이 필요합니다:  pip install pyyaml")

TARGET_VERSION = 5

# version 5 topic_metadata가 아는 키
V5_TOPIC_KEYS = {"name", "type", "serialization_format", "offered_qos_profiles"}
# version 5 최상위에서 아는 키
V5_TOP_KEYS = {
    "version", "storage_identifier", "duration", "starting_time",
    "message_count", "topics_with_message_count",
    "compression_format", "compression_mode",
    "relative_file_paths", "files",
}
V5_FILE_KEYS = {"path", "starting_time", "duration", "message_count"}


def patch(info, verbose=True):
    changes = []

    old_ver = info.get("version")
    if old_ver != TARGET_VERSION:
        info["version"] = TARGET_VERSION
        changes.append(f"version: {old_ver} -> {TARGET_VERSION}")

    for entry in info.get("topics_with_message_count", []) or []:
        tm = entry.get("topic_metadata", {})
        name = tm.get("name", "?")

        qos = tm.get("offered_qos_profiles")
        if not isinstance(qos, str):
            # 시퀀스(대개 빈 리스트) -> 빈 문자열.
            # 빈 문자열은 "QoS 오버라이드 없음"으로 해석되어 재생에 지장이 없다.
            tm["offered_qos_profiles"] = ""
            changes.append(f"{name}: offered_qos_profiles {type(qos).__name__} -> str")

        for k in list(tm):
            if k not in V5_TOPIC_KEYS:
                tm.pop(k)
                changes.append(f"{name}: '{k}' 제거")

    for f in info.get("files", []) or []:
        for k in list(f):
            if k not in V5_FILE_KEYS:
                f.pop(k)
                changes.append(f"files[{f.get('path','?')}]: '{k}' 제거")

    for k in list(info):
        if k not in V5_TOP_KEYS:
            info.pop(k)
            changes.append(f"최상위 '{k}' 제거")

    return changes


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 디렉터리 (metadata.yaml이 있는 곳)")
    ap.add_argument("--dry-run", action="store_true",
                    help="변경 내용만 출력하고 쓰지 않음")
    args = ap.parse_args()

    bag = Path(args.bag).expanduser()
    meta = bag / "metadata.yaml" if bag.is_dir() else bag
    if not meta.is_file():
        sys.exit(f"metadata.yaml을 찾을 수 없습니다: {meta}")

    with meta.open("r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)

    root_key = "rosbag2_bagfile_information"
    if root_key not in doc:
        sys.exit(f"'{root_key}' 키가 없습니다. rosbag2 메타데이터가 맞습니까?")

    info = doc[root_key]
    print(f"대상: {meta}")
    print(f"현재 version: {info.get('version')}\n")

    changes = patch(info)

    if not changes:
        print("변경할 내용이 없습니다. 이미 호환 형식입니다.")
        return

    print("변경 내역:")
    for c in changes:
        print(f"  - {c}")

    if args.dry_run:
        print("\n--dry-run 이므로 저장하지 않았습니다.")
        return

    bak = meta.with_suffix(".yaml.bak")
    if not bak.exists():
        shutil.copy2(meta, bak)
        print(f"\n백업: {bak}")

    with meta.open("w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, default_flow_style=False,
                       sort_keys=False, allow_unicode=True)

    print(f"저장 완료: {meta}")
    print(f"\n확인:  ros2 bag info {bag}")


if __name__ == "__main__":
    main()
