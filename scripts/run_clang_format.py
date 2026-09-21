import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

EXTENSIONS = (".c", ".cpp", ".h", ".hpp")
# .claudeはエージェント用worktree(リポジトリのコピー)を含むため対象外
EXCLUDE_DIRS = {"build", ".git", "dist", "ext", "assets", ".cache", ".claude"}
# 外部リポジトリ(AviUtlスクリプトのsubmodule)は整形しない
EXCLUDE_PATHS = {os.path.join(".", "plugins", "scripts", "Aodaruma-AviUtl-Script")}
CHECK = "--check" in sys.argv[1:]

paths = []
for root, dirs, files in os.walk("."):
    dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS and os.path.join(root, d) not in EXCLUDE_PATHS]
    paths += [os.path.join(root, f) for f in files if f.endswith(EXTENSIONS)]


def run(path):
    cmd = ["clang-format", "--dry-run", "--Werror", path] if CHECK else ["clang-format", "-i", path]
    r = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    return path, r


failed = []
# clang-formatは別プロセスなので、スレッドで並列に起動すればよい(GILの影響なし)
with ThreadPoolExecutor(max_workers=8) as ex:
    for path, r in ex.map(run, paths):
        if CHECK:
            if r.returncode != 0:
                failed.append(path)
                print(r.stderr, end="")
        else:
            print("Formatting:", path)

if CHECK and failed:
    print(f"\n{len(failed)}個のファイルがclang-format未適用です。`just format`を実行してください。")
    sys.exit(1)
