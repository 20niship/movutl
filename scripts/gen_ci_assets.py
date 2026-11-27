#!/usr/bin/env python3
"""非公開submodule assets/ を取得できない環境(CI)向けに、テストが参照する素材を合成して作る。

使い方: python3 scripts/gen_ci_assets.py [出力ディレクトリ(既定: assets)]
必要: ffmpeg、日本語グリフを持つTrueTypeフォント(環境変数 MU_CI_FONT。既定はfonts-ipafont-gothic)
本物の素材を上書きしないよう、出力先が空でなければ何もせず失敗する。
解像度・fps・長さは本物に合わせてある。
"""
import os
import shutil
import subprocess
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(root, "assets")
if os.path.isdir(out) and os.listdir(out):
    sys.exit(f"{out} は空ではありません。本物のassetsは上書きしません")
for d in ("audio", "movies", "textures", "images", "fonts", "brushes"):
    os.makedirs(os.path.join(out, d), exist_ok=True)


def ffmpeg(*args):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *args], check=True)


def lavfi(src):
    return ["-f", "lavfi", "-i", src]


def path(name):
    return os.path.join(out, name)


# export_plugin_testが赤矩形/緑円を色で探すので背景は無彩色にし、隅がベタ塗りにならないようノイズを乗せる
bg = "hue=s=0,noise=alls=30:allf=t"
x264 = ["-c:v", "libx264", "-pix_fmt", "yuv420p"]
ffmpeg(*lavfi(f"testsrc2=size=640x360:rate=30,{bg}"), "-frames:v", "300", *x264, path("movies/big_buck_bunny_360_10s.mp4"))
ffmpeg(*lavfi(f"testsrc=size=1920x1080:rate=30,{bg}"), *lavfi("sine=frequency=440:sample_rate=44100"), "-frames:v", "171", "-t", "5.759", "-ac", "2", *x264, "-c:a", "aac", path("movies/sample-5s.mp4"))
ffmpeg(*lavfi("sine=frequency=440:sample_rate=44100:duration=5.943"), "-ac", "2", "-c:a", "pcm_s16le", path("audio/file_example_WAV_1MG.wav"))

images = {
    "textures/tile.jpg": "512x512",
    "textures/warning.jpeg": "128x128",
    "textures/concrete.png": "512x512",
    "textures/wood.png": "512x512",
    "textures/concrete1.png": "512x512",
    "textures/concrete4.png": "512x512",
    "images/blender_png.png": "626x196",
    "images/texture.jpg": "347x346",
    "images/folder.jpg": "400x400",
}
for name, size in images.items():
    fmt = ["-pix_fmt", "rgba"] if name.endswith(".png") and name != "textures/concrete.png" else []
    ffmpeg(*lavfi(f"testsrc2=size={size}"), "-frames:v", "1", *fmt, path(name))

font = os.environ.get("MU_CI_FONT", "/usr/share/fonts/opentype/ipafont-gothic/ipag.ttf")
shutil.copy(font, path("fonts/Meiryo.ttf"))
subprocess.run([sys.executable, os.path.join(root, "scripts", "gen_brush_assets.py"), path("brushes")], check=True)
print(f"generated: {out}")
