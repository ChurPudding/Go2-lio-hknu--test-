#!/usr/bin/env python3
"""
격자지도에서 거리 재기

grid.pgm 을 띄우고 두 점을 클릭하면 그 사이 거리를 m 로 출력한다.
실측값을 주면 지도 축척 오차와 보정계수까지 계산한다.

사용 예
  python3 map_measure.py ~/fastlio_ws/results/odommap_v2/grid.yaml
  python3 map_measure.py ~/fastlio_ws/results/odommap_v2/grid.yaml --actual 21.84

조작
  좌클릭 2회   한 구간 측정. 계속 반복 가능
  r            찍은 점 초기화
  q            종료
  돋보기 도구로 확대한 뒤 클릭하면 정밀하게 찍을 수 있다.
"""

import argparse
import math
import os
import sys

import warnings

import numpy as np
import yaml
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import font_manager

warnings.filterwarnings("ignore")

_HAVE_KR = False
for _name in ("NanumGothic", "NanumBarunGothic", "Noto Sans CJK KR", "Malgun Gothic"):
    if any(f.name == _name for f in font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _name
        matplotlib.rcParams["axes.unicode_minus"] = False
        _HAVE_KR = True
        break

TITLE_KR = "두 점을 클릭하면 거리가 터미널에 출력됩니다   (r 초기화 / q 종료)"
TITLE_EN = "click two points to measure   (r = reset, q = quit)"
TITLE = TITLE_KR if _HAVE_KR else TITLE_EN


def read_pgm(path):
    """P5(이진) / P2(텍스트) PGM 파서. Pillow 없이 동작한다."""
    with open(path, "rb") as f:
        data = f.read()

    toks, idx = [], 0
    while len(toks) < 4:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx:idx + 1] not in (b"\n", b"\r"):
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        toks.append(data[start:idx])

    magic = toks[0].decode()
    w, h, maxv = int(toks[1]), int(toks[2]), int(toks[3])
    idx += 1

    if magic == "P5":
        dtype = np.uint8 if maxv < 256 else ">u2"
        img = np.frombuffer(data, dtype=dtype, count=w * h, offset=idx)
    elif magic == "P2":
        img = np.array(data[idx:].split(), dtype=np.int32)[:w * h]
    else:
        raise ValueError(f"지원하지 않는 형식입니다: {magic}")

    return img.reshape(h, w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml_path", help="grid.yaml 경로")
    ap.add_argument("--actual", type=float, default=None, help="실측 거리 [m]")
    ap.add_argument("--res", type=float, default=None,
                    help="해상도 직접 지정 [m/cell]. yaml 에 없을 때만")
    args = ap.parse_args()

    with open(args.yaml_path) as f:
        meta = yaml.safe_load(f)

    res = args.res or meta.get("resolution")
    if res is None:
        sys.exit("yaml 에 resolution 이 없습니다. --res 로 직접 주십시오.")

    pgm = meta.get("image", "grid.pgm")
    if not os.path.isabs(pgm):
        pgm = os.path.join(os.path.dirname(os.path.abspath(args.yaml_path)), pgm)

    img = read_pgm(pgm)
    print(f"{os.path.basename(pgm)}  {img.shape[1]} x {img.shape[0]} px"
          f"  ·  해상도 {res} m/cell"
          f"  ·  전체 {img.shape[1]*res:.2f} x {img.shape[0]*res:.2f} m")
    if args.actual:
        print(f"실측 기준 {args.actual:.2f} m\n")
    else:
        print()

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.imshow(img, cmap="gray", origin="upper", interpolation="nearest")
    ax.set_title(TITLE)

    pts = []
    n = [0]

    def onclick(e):
        if e.inaxes is not ax or e.xdata is None:
            return
        if fig.canvas.toolbar.mode:      # 확대·이동 모드일 때는 무시
            return
        pts.append((e.xdata, e.ydata))
        ax.plot(e.xdata, e.ydata, "+", color="red", ms=12, mew=1.5)

        if len(pts) == 2:
            (x0, y0), (x1, y1) = pts
            ax.plot([x0, x1], [y0, y1], "-", color="red", lw=1.2)
            d_px = math.hypot(x1 - x0, y1 - y0)
            d_m = d_px * res
            n[0] += 1

            print(f"[{n[0]}] ({x0:.1f}, {y0:.1f}) → ({x1:.1f}, {y1:.1f})")
            print(f"     {d_px:.1f} px  ×  {res}  =  지도 거리 {d_m:.3f} m")
            if args.actual:
                err = (d_m / args.actual - 1) * 100
                k = args.actual / d_m
                word = "짧습니다" if err < 0 else "깁니다"
                print(f"     실측 {args.actual:.2f} m 대비 {abs(err):.2f}% {word}"
                      f"   ·  보정계수 {k:.4f}")
            print()

            ax.annotate(f"{d_m:.2f} m", ((x0 + x1) / 2, (y0 + y1) / 2),
                        color="red", fontsize=10,
                        xytext=(6, 6), textcoords="offset points")
            pts.clear()

        fig.canvas.draw_idle()

    def onkey(e):
        if e.key == "r":
            pts.clear()
            n[0] = 0
            ax.cla()
            ax.imshow(img, cmap="gray", origin="upper", interpolation="nearest")
            ax.set_title(TITLE)
            fig.canvas.draw_idle()
            print("초기화했습니다.\n")
        elif e.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", onclick)
    fig.canvas.mpl_connect("key_press_event", onkey)
    fig.subplots_adjust(left=0.06, right=0.98, top=0.94, bottom=0.06)
    plt.show()


if __name__ == "__main__":
    main()
