#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_pcd.py — 만들어 둔 점군(.pcd) 전부를 한 장에 나란히 놓는다

results/outdoor_0812/*.pcd 를 모두 찾아 top view 로 그린다.
모든 패널의 표시 폭을 같게 맞추고 20 m 축척 막대를 넣어
bag 별·보정방식별 크기와 형태를 눈으로 바로 비교할 수 있게 한다.

사용
----
  python3 compare_pcd.py                       # 전부
  python3 compare_pcd.py --view front          # 정면(x-z)
  python3 compare_pcd.py --match '*1449*'      # 특정 bag 만
  python3 compare_pcd.py --span 200            # 표시 폭 고정
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

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib 이 없습니다.")

BASE = os.path.expanduser("~/fastlio_ws/results/outdoor_0812")

# 보정 방식별 테두리 색
MODE_COLOR = {"none": "tab:gray", "rigid": "tab:blue", "warp": "tab:green"}


def label_of(path):
    n = os.path.splitext(os.path.basename(path))[0]
    return n.replace("go2_", "").replace("_0812", "")


def mode_of(path):
    for m in ("warp", "rigid", "none"):
        if path.endswith(f"_{m}.pcd"):
            return m
    return ""


def sort_key(path):
    n = os.path.basename(path)
    order = {"none": 0, "rigid": 1, "warp": 2}
    return (n.split("_none")[0].split("_rigid")[0].split("_warp")[0],
            order.get(mode_of(path), 9))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--match", default="*.pcd")
    ap.add_argument("--view", default="top", choices=["top", "front", "side"])
    ap.add_argument("--span", type=float, default=None)
    ap.add_argument("--cols", type=int, default=0)
    ap.add_argument("--max-points", type=int, default=120000)
    ap.add_argument("--point-size", type=float, default=0.12)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    files = sorted(
        (f for f in glob.glob(os.path.join(args.base, "*.pcd"))
         if fnmatch.fnmatch(os.path.basename(f), args.match)),
        key=sort_key)
    if not files:
        sys.exit(f"pcd 를 찾지 못했습니다: {args.base}/{args.match}")

    ai, bi = {"top": (0, 1), "front": (0, 2), "side": (1, 2)}[args.view]
    alab, blab = [("x [m]", "y [m]"), ("x [m]", "z [m]"), ("y [m]", "z [m]")][
        ["top", "front", "side"].index(args.view)]

    panels = []
    for f in files:
        P = np.asarray(o3d.io.read_point_cloud(f).points)
        if P.size == 0:
            print(f"[건너뜀] {os.path.basename(f)} — 비어 있음")
            continue
        n_all = len(P)
        if n_all > args.max_points:
            idx = np.random.default_rng(0).choice(n_all, args.max_points, replace=False)
            Q = P[idx]
        else:
            Q = P
        a, b = Q[:, ai], Q[:, bi]
        lo = np.array([P[:, ai].min(), P[:, bi].min()])
        hi = np.array([P[:, ai].max(), P[:, bi].max()])
        panels.append(dict(name=label_of(f), mode=mode_of(f), a=a, b=b,
                           z=Q[:, 2], n=n_all,
                           c=(lo + hi) / 2.0, span=float((hi - lo).max())))
        print(f"{label_of(f):<36} {n_all:>9,} 점   "
              f"{alab[0]} {hi[0]-lo[0]:6.1f} m   {blab[0]} {hi[1]-lo[1]:6.1f} m")

    span = args.span if args.span else max(p["span"] for p in panels) * 1.05
    zlo = min(p["z"].min() for p in panels)
    zhi = max(np.percentile(p["z"], 99) for p in panels)

    n = len(panels)
    cols = args.cols if args.cols else min(4, n)
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4.6 * cols, 4.9 * rows))
    axes = np.atleast_1d(axes).ravel()

    sc = None
    for ax, p in zip(axes, panels):
        sc = ax.scatter(p["a"], p["b"], c=p["z"], s=args.point_size,
                        cmap="viridis", vmin=zlo, vmax=zhi, linewidths=0)
        ax.set_xlim(p["c"][0] - span / 2, p["c"][0] + span / 2)
        ax.set_ylim(p["c"][1] - span / 2, p["c"][1] + span / 2)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25, ls=":", lw=0.5)
        ax.set_xlabel(alab, fontsize=8)
        ax.set_ylabel(blab, fontsize=8)
        ax.tick_params(labelsize=7)
        ax.set_title(f"{p['name']}\n{p['n']:,} pts", fontsize=9)

        col = MODE_COLOR.get(p["mode"], "black")
        for s in ax.spines.values():
            s.set_edgecolor(col)
            s.set_linewidth(2.0)

        x0 = p["c"][0] - span / 2 + span * 0.06
        y0 = p["c"][1] - span / 2 + span * 0.06
        ax.plot([x0, x0 + 20], [y0, y0], "-", lw=3, color="tab:red")
        ax.text(x0 + 10, y0 + span * 0.02, "20 m", ha="center",
                fontsize=7, color="tab:red")

    for ax in axes[n:]:
        ax.axis("off")

    if sc is not None:
        cb = fig.colorbar(sc, ax=axes[:n].tolist(), shrink=0.6, pad=0.01)
        cb.set_label("z [m]", fontsize=8)
        cb.ax.tick_params(labelsize=7)

    handles = [plt.Line2D([], [], color=c, lw=3, label=m)
               for m, c in MODE_COLOR.items()]
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=9,
               frameon=False, title="frame color = correction mode",
               title_fontsize=8)

    fig.suptitle(f"Outdoor 0812 point clouds — {args.view} view "
                 f"(all panels span {span:.0f} m)", fontsize=13)

    out = args.out or os.path.join(args.base, f"compare_pcd_{args.view}.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"\n저장: {out}")


if __name__ == "__main__":
    main()
