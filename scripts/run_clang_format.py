import subprocess
import sys
import os

EXTENSIONS = (".c", ".cpp", ".h", ".hpp")
EXCLUDE_DIRS = {"build", ".git", "dist", "ext", "assets", ".cache"}
CHECK = "--check" in sys.argv[1:]

failed = []
for root, dirs, files in os.walk("."):
    dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]

    for file in files:
        if not file.endswith(EXTENSIONS):
            continue
        path = os.path.join(root, file)
        if CHECK:
            result = subprocess.run(["clang-format", "--dry-run", "--Werror", path], capture_output=True, text=True, errors="replace")
            if result.returncode != 0:
                failed.append(path)
                print(result.stderr, end="")
        else:
            print("Formatting:", path)
            subprocess.run(["clang-format", "-i", path])

if CHECK and failed:
    print(f"\n{len(failed)}個のファイルがclang-format未適用です。`just format`を実行してください。")
    sys.exit(1)
