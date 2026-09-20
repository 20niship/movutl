#!/usr/bin/env python3
"""tim3.web.fc2.com/sidx.htm の DL 列の zip を全て取得し、解凍して plugins/scripts/aviutl/ に格納する。"""
import io
import re
import sys
import urllib.request
import zipfile
from pathlib import Path
from urllib.parse import unquote, urljoin

INDEX = "https://tim3.web.fc2.com/sidx.htm"
OUT = Path(__file__).resolve().parent.parent / "plugins" / "scripts" / "aviutl"


def get(url: str) -> bytes:
  req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
  with urllib.request.urlopen(req, timeout=60) as r:
    return r.read()


def main() -> int:
  html = get(INDEX).decode("shift_jis", errors="replace")
  urls = sorted({urljoin(INDEX, m) for m in re.findall(r'href="([^"]+\.zip)"', html, re.I)})
  print(f"{len(urls)} zip found")
  OUT.mkdir(parents=True, exist_ok=True)
  failed = 0
  for url in urls:
    name = Path(unquote(url)).stem
    try:
      with zipfile.ZipFile(io.BytesIO(get(url))) as z:
        for info in z.infolist():
          # 日本語ファイル名は cp437 で読まれるので cp932 に直す
          fn = info.filename if info.flag_bits & 0x800 else info.filename.encode("cp437").decode("cp932", errors="replace")
          dst = (OUT / name / fn).resolve()
          if not dst.is_relative_to(OUT.resolve()):  # zip slip 対策
            continue
          if info.is_dir():
            dst.mkdir(parents=True, exist_ok=True)
          else:
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_bytes(z.read(info))
      print(f"ok   {name}")
    except Exception as e:  # ponytail: 1件失敗しても続行、終了コードで通知
      failed += 1
      print(f"FAIL {url}: {e}", file=sys.stderr)
  return 1 if failed else 0


if __name__ == "__main__":
  sys.exit(main())
