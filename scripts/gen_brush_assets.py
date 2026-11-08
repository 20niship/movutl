#!/usr/bin/env python3
"""assets/brushes/ のブラシ先端・紙目・パターン画像を生成する(標準ライブラリのみ)。

使い方: python3 scripts/gen_brush_assets.py [出力ディレクトリ(既定: assets/brushes)]
先端はPhotoshop規約(黒=描く、アルファで形)のRGBA、テクスチャ/パターンはグレースケールRGBA。乱数は固定seedなので毎回同じ画像になる。
"""
import math
import os
import random
import struct
import sys
import zlib


def write_png(path, w, h, px):
    """px: h行の[(r,g,b,a)*w]。"""
    raw = b"".join(b"\x00" + bytes(c for p in row for c in p) for row in px)

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
    data += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(data)


def tip(w, h, alpha_fn):
    """黒でアルファだけ変える先端画像。alpha_fn(u,v)->0..1 (u,v は -1..1)。"""
    rows = []
    for j in range(h):
        row = []
        for i in range(w):
            u, v = (i + 0.5) / w * 2 - 1, (j + 0.5) / h * 2 - 1
            a = max(0.0, min(1.0, alpha_fn(u, v)))
            row.append((0, 0, 0, int(a * 255 + 0.5)))
        rows.append(row)
    return rows


def gray(w, h, fn):
    rows = []
    for j in range(h):
        row = []
        for i in range(w):
            g = int(max(0.0, min(1.0, fn(i, j))) * 255 + 0.5)
            row.append((g, g, g, 255))
        rows.append(row)
    return rows


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "assets", "brushes")
    os.makedirs(out, exist_ok=True)
    rnd = random.Random(1234)

    # --- 先端 ---
    write_png(os.path.join(out, "tip_soft_round.png"), 64, 64, tip(64, 64, lambda u, v: 1 - math.hypot(u, v)))
    write_png(os.path.join(out, "tip_square.png"), 64, 64, tip(64, 64, lambda u, v: 1.0 if max(abs(u), abs(v)) < 0.95 else 0.0))
    write_png(os.path.join(out, "tip_flat.png"), 64, 64, tip(64, 64, lambda u, v: 1.0 if (abs(u) < 0.95 and abs(v) < 0.25) else 0.0))
    write_png(os.path.join(out, "tip_star.png"), 64, 64, tip(64, 64, lambda u, v: 1.0 if math.hypot(u, v) < 0.95 * (0.55 + 0.45 * abs(math.cos(2.5 * math.atan2(v, u)))) else 0.0))
    # チョーク: 円の中にランダムな欠け
    chalk = [[rnd.random() for _ in range(64)] for _ in range(64)]
    write_png(os.path.join(out, "tip_chalk.png"), 64, 64,
              tip(64, 64, lambda u, v: (1.0 if chalk[min(63, int((v + 1) * 32))][min(63, int((u + 1) * 32))] > 0.35 else 0.0) * (1 if math.hypot(u, v) < 0.95 else 0.0)))
    # 毛筆: 縦に並んだ毛の筋
    bristle = [rnd.random() for _ in range(64)]
    write_png(os.path.join(out, "tip_bristle.png"), 64, 64,
              tip(64, 64, lambda u, v: (0.4 + 0.6 * bristle[min(63, int((u + 1) * 32))]) * (1 if math.hypot(u, v) < 0.95 else 0.0)))

    # --- 紙目(白=よく乗る) ---
    n = 128
    noise = [[rnd.random() for _ in range(n)] for _ in range(n)]
    write_png(os.path.join(out, "paper_grain.png"), n, n, gray(n, n, lambda i, j: 0.35 + 0.65 * noise[j][i]))
    write_png(os.path.join(out, "canvas_weave.png"), n, n, gray(n, n, lambda i, j: 0.55 + 0.25 * math.sin(i * math.pi / 4) + 0.2 * math.sin(j * math.pi / 4)))

    # --- パターン(テクスチャ貼り用) ---
    write_png(os.path.join(out, "pattern_check.png"), 32, 32, gray(32, 32, lambda i, j: 0.15 if ((i // 8) + (j // 8)) % 2 == 0 else 0.9))
    write_png(os.path.join(out, "pattern_stripe.png"), 32, 32, gray(32, 32, lambda i, j: 0.1 if ((i + j) // 8) % 2 == 0 else 0.85))
    write_png(os.path.join(out, "pattern_dots.png"), 32, 32, gray(32, 32, lambda i, j: 0.1 if math.hypot((i % 16) - 8, (j % 16) - 8) < 4 else 0.9))
    print("wrote brush assets to", os.path.abspath(out))


if __name__ == "__main__":
    main()
