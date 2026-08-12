#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
view3d.py — 만들어 둔 점군들을 Open3D 창에서 3D 로 본다

모드 세 가지
------------
grid     13개를 바둑판처럼 펼쳐 한 화면에 모두 띄운다. 전체 조망용.
single   하나씩 순서대로 띄운다. 창을 닫으면 다음 것으로 넘어간다. 자세히 볼 때.
overlay  여러 개를 같은 원점에 겹쳐 띄운다. none/rigid/warp 정렬 비교용.

조작
----
  마우스 왼쪽 드래그   회전
  마우스 휠           확대·축소
  마우스 휠 드래그     이동
  R                   시점 초기화
  H                   도움말 (터미널에 출력)
  Q 또는 ESC          닫기 (single 모드는 다음으로)

사용
----
  python3 view3d.py                                  # grid, 전부
  python3 view3d.py --mode single --match '*1449*'
  python3 view3d.py --mode overlay --match '*1449*'  # 세 방식 겹쳐보기
  python3 view3d.py --color height                   # 높이 색상 (기본)
  python3 view3d.py --color file                     # 파일별 단색
"""

import argparse
import fnmatch
import glob
import os
import sys

import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d 가 없습니다.")

BASE = os.path.expanduser("~/fastlio_ws/results/outdoor_0812")

PALETTE = [
    (0.90, 0.29, 0.23), (0.20, 0.60, 0.86), (0.18, 0.80, 0.44),
    (0.95, 0.77, 0.06), (0.61, 0.35, 0.71), (0.90, 0.49, 0.13),
    (0.10, 0.74, 0.61), (0.93, 0.35, 0.61), (0.40, 0.40, 0.40),
    (0.55, 0.76, 0.29), (0.20, 0.29, 0.37), (0.85, 0.65, 0.13),
    (0.35, 0.70, 0.90),
]


def label_of(path):
    n = os.path.splitext(os.path.basename(path))[0]
    return n.replace("go2_", "").replace("_0812", "")


def sort_key(path):
    n = os.path.basename(path)
    order = {"none": 0, "rigid": 1, "warp": 2}
    m = next((k for k in order if n.endswith(f"_{k}.pcd")), "z")
    base = n.split("_none")[0].split("_rigid")[0].split("_warp")[0]
    return (base, order.get(m, 9))


def height_colors(P, lo=None, hi=None):
    z = P[:, 2]
    lo = np.percentile(z, 1) if lo is None else lo
    hi = np.percentile(z, 99) if hi is None else hi
    t = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1)
    # viridis 근사 (남색 -> 청록 -> 노랑)
    r = np.clip(-0.3 + 1.6 * t ** 2, 0, 1)
    g = np.clip(0.1 + 0.85 * t, 0, 1)
    b = np.clip(0.55 + 0.5 * t - 1.3 * t ** 2, 0, 1)
    return np.column_stack([r, g, b])


def load(path, color_mode, idx, zrange=None):
    pcd = o3d.io.read_point_cloud(path)
    P = np.asarray(pcd.points)
    if P.size == 0:
        return None, None
    if color_mode == "height":
        lo, hi = (zrange if zrange else (None, None))
        pcd.colors = o3d.utility.Vector3dVector(height_colors(P, lo, hi))
    else:
        c = np.tile(PALETTE[idx % len(PALETTE)], (len(P), 1))
        pcd.colors = o3d.utility.Vector3dVector(c)
    return pcd, P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--match", default="*.pcd")
    ap.add_argument("--mode", default="grid", choices=["grid", "single", "overlay"])
    ap.add_argument("--color", default="height", choices=["height", "file"])
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--gap", type=float, default=30.0,
                    help="grid 모드에서 지도 사이 여백 m")
    ap.add_argument("--axis", type=float, default=10.0,
                    help="좌표축 크기 m. 0 이면 표시 안 함")
    args = ap.parse_args()

    files = sorted(
        (f for f in glob.glob(os.path.join(args.base, "*.pcd"))
         if fnmatch.fnmatch(os.path.basename(f), args.match)),
        key=sort_key)
    if not files:
        sys.exit(f"pcd 를 찾지 못했습니다: {args.base}/{args.match}")

    print(f"{len(files)}개:")
    for f in files:
        print("  " + label_of(f))

    # 높이 색상 범위를 전체에서 공통으로 잡는다
    zlo, zhi = [], []
    for f in files:
        P = np.asarray(o3d.io.read_point_cloud(f).points)
        if P.size:
            zlo.append(np.percentile(P[:, 2], 1))
            zhi.append(np.percentile(P[:, 2], 99))
    zrange = (min(zlo), max(zhi)) if zlo else None

    # ---------------- single ----------------
    if args.mode == "single":
        for i, f in enumerate(files):
            pcd, P = load(f, args.color, i, zrange)
            if pcd is None:
                continue
            geos = [pcd]
            if args.axis > 0:
                geos.append(o3d.geometry.TriangleMesh.create_coordinate_frame(
                    size=args.axis, origin=P.min(0)))
            print(f"\n[{i+1}/{len(files)}] {label_of(f)}  "
                  f"({len(P):,} 점)   창을 닫으면 다음")
            o3d.visualization.draw_geometries(
                geos, window_name=f"{i+1}/{len(files)}  {label_of(f)}",
                width=1280, height=860)
        return

    # ---------------- overlay ----------------
    if args.mode == "overlay":
        geos = []
        for i, f in enumerate(files):
            pcd, P = load(f, "file" if args.color == "height" else args.color,
                          i, zrange)
            if pcd is None:
                continue
            geos.append(pcd)
            print(f"  {PALETTE[i % len(PALETTE)]}  {label_of(f)}")
        if args.axis > 0:
            geos.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=args.axis))
        print("\n색상 = 파일 구분. 잘 겹치면 정렬이 일치하는 것입니다.")
        o3d.visualization.draw_geometries(geos, window_name="overlay",
                                          width=1400, height=900)
        return

    # ---------------- grid ----------------
    loaded = []
    for i, f in enumerate(files):
        pcd, P = load(f, args.color, i, zrange)
        if pcd is None:
            continue
        loaded.append((f, pcd, P))

    cell = max(max(P.max(0)[:2] - P.min(0)[:2]) for _, _, P in loaded) + args.gap
    cols = args.cols
    geos = []
    print(f"\n한 칸 {cell:.0f} m, {cols}열")
    for i, (f, pcd, P) in enumerate(loaded):
        r, c = divmod(i, cols)
        # 각 점군을 자기 최소점 기준으로 옮긴 뒤 칸 위치로 배치
        base = P.min(0)
        off = np.array([c * cell - base[0],
                        -r * cell - base[1],
                        -base[2]])
        pcd.translate(off)
        geos.append(pcd)
        if args.axis > 0:
            geos.append(o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=args.axis, origin=[c * cell, -r * cell, 0.0]))
        print(f"  ({r},{c})  {label_of(f)}")

    print("\n좌상단부터 오른쪽으로 정렬. 각 칸의 좌표축이 그 지도의 원점입니다.")
    o3d.visualization.draw_geometries(geos, window_name="outdoor 0812 — grid",
                                      width=1500, height=950)


if __name__ == "__main__":
    main()
