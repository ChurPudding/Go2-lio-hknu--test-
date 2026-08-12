#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_maps.py — 만들어 둔 격자지도들을 한 장으로 비교한다

grid*_<이름>/map.pgm + map.yaml 을 찾아 패널로 나란히 그린다.
각 패널은 실제 미터 좌표로 그리고, 모든 패널의 표시 폭을 같게 맞춰
크기·형태를 눈으로 직접 비교할 수 있게 한다.

사용
----
  python3 compare_maps.py                      # results/outdoor_0812 전체
  python3 compare_maps.py --dirs grid15_*      # 특정 폴더만
  python3 compare_maps.py --span 160           # 표시 폭 160 m 로 고정
"""

import argparse
import glob
import os
import re
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib 이 없습니다.")

BASE = os.path.expanduser("~/fastlio_ws/results/outdoor_0812")


def read_pgm(path):
    """P5 / P2 PGM 을 (H,W) uint8/uint16 배열로."""
    with open(path, "rb") as f:
        data = f.read()

    # 주석을 걸러가며 토큰 3개(magic 제외: w,h,maxval) 를 읽는다
    pos = 0

    def token():
        nonlocal pos
        while True:
            while pos < len(data) and data[pos:pos + 1].isspace():
                pos += 1
            if pos < len(data) and data[pos:pos + 1] == b"#":
                while pos < len(data) and data[pos:pos + 1] not in (b"\n", b"\r"):
                    pos += 1
            else:
                break
        s = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        return data[s:pos]

    magic = token()
    w = int(token())
    h = int(token())
    maxv = int(token())
    pos += 1  # 단일 공백

    if magic == b"P5":
        dt = np.uint8 if maxv < 256 else ">u2"
        arr = np.frombuffer(data, dtype=dt, count=w * h, offset=pos)
    elif magic == b"P2":
        arr = np.array(data[pos:].split(), dtype=int)[:w * h]
    else:
        raise ValueError(f"지원하지 않는 형식: {magic}")
    return arr.reshape(h, w)


def read_yaml(path):
    """resolution 과 origin 만 뽑는다 (yaml 모듈 없이도 동작)."""
    res, ox, oy = 0.05, 0.0, 0.0
    with open(path) as f:
        for ln in f:
            if ln.startswith("resolution"):
                res = float(ln.split(":")[1])
            elif ln.startswith("origin"):
                nums = re.findall(r"-?\d+\.?\d*(?:[eE][-+]?\d+)?", ln.split(":", 1)[1])
                if len(nums) >= 2:
                    ox, oy = float(nums[0]), float(nums[1])
    return res, ox, oy


def label_of(dirname):
    """폴더명을 짧은 제목으로."""
    n = os.path.basename(dirname)
    n = re.sub(r"^grid\d*_", "", n)
    n = n.replace("go2_", "").replace("_0812", "")
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--dirs", nargs="*", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--span", type=float, default=None,
                    help="각 패널의 표시 폭 m. 미지정 시 가장 큰 지도에 맞춘다")
    ap.add_argument("--cols", type=int, default=0)
    args = ap.parse_args()

    if args.dirs:
        dirs = []
        for d in args.dirs:
            dirs += glob.glob(d if os.path.isabs(d) else os.path.join(args.base, d))
    else:
        dirs = glob.glob(os.path.join(args.base, "grid*_*"))
    dirs = sorted(d for d in dirs
                  if os.path.exists(os.path.join(d, "map.pgm"))
                  and os.path.exists(os.path.join(d, "map.yaml")))
    if not dirs:
        sys.exit(f"지도를 찾지 못했습니다: {args.base}")

    panels = []
    for d in dirs:
        try:
            img = read_pgm(os.path.join(d, "map.pgm"))
            res, ox, oy = read_yaml(os.path.join(d, "map.yaml"))
        except Exception as e:
            print(f"[건너뜀] {d}: {e}")
            continue
        h, w = img.shape
        # 점유 = 어두운 칸. ROS 규약상 pgm 은 위아래가 뒤집혀 저장된다.
        occ = img < 128
        extent = (ox, ox + w * res, oy, oy + h * res)
        cx, cy = ox + w * res / 2.0, oy + h * res / 2.0
        panels.append(dict(name=label_of(d), img=occ[::-1], extent=extent,
                           res=res, w=w, h=h, cx=cx, cy=cy,
                           span=max(w * res, h * res),
                           occ=int(occ.sum())))
        print(f"{label_of(d):<34} {w}x{h}  res {res:.3f}  점유 {int(occ.sum()):,}")

    span = args.span if args.span else max(p["span"] for p in panels) * 1.05

    n = len(panels)
    cols = args.cols if args.cols else min(3, n)
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5.4 * cols, 5.8 * rows))
    axes = np.atleast_1d(axes).ravel()

    for ax, p in zip(axes, panels):
        ax.imshow(p["img"], cmap="gray_r", origin="lower",
                  extent=p["extent"], interpolation="nearest",
                  vmin=0, vmax=1)
        ax.set_xlim(p["cx"] - span / 2, p["cx"] + span / 2)
        ax.set_ylim(p["cy"] - span / 2, p["cy"] + span / 2)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25, ls=":", lw=0.5)
        ax.set_title(f"{p['name']}\n{p['w']}x{p['h']} @ {p['res']*100:.0f} cm/px"
                     f"   occ {p['occ']:,}", fontsize=9)
        ax.tick_params(labelsize=7)

        # 20 m 축척 막대
        x0 = p["cx"] - span / 2 + span * 0.06
        y0 = p["cy"] - span / 2 + span * 0.06
        ax.plot([x0, x0 + 20], [y0, y0], "-", lw=3, color="tab:red")
        ax.text(x0 + 10, y0 + span * 0.015, "20 m", ha="center",
                fontsize=7, color="tab:red")

    for ax in axes[n:]:
        ax.axis("off")

    fig.suptitle(f"Occupancy grid comparison  (all panels span {span:.0f} m)",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.985))

    out = args.out or os.path.join(args.base, "compare_maps.png")
    fig.savefig(out, dpi=140)
    print(f"\n저장: {out}")


if __name__ == "__main__":
    main()
