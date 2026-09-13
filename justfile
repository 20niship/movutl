default:
    @just --list

# cmake configure + ビルド
build build_dir="build":
    git submodule update --init --recursive
    cmake -S . -B {{build_dir}}
    cmake --build {{build_dir}} -j${BUILD_JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu)}

# ビルドしてmovutl_mainを実行する(例: just run ./examples/foobar.lua)
# movutl_mainは../assets等相対パスでリソースを解決するためbuild/から実行する必要がある
run *args: build
    #!/usr/bin/env bash
    set -euo pipefail
    args=({{args}})
    if [ ${#args[@]} -gt 0 ]; then
      args[0]="$(cd "$(dirname "${args[0]}")" && pwd)/$(basename "${args[0]}")"
    fi
    cd build && exec ./movutl_main "${args[@]+"${args[@]}"}"

# ビルドしてテストを実行する
test: build
    ctest --test-dir build --output-on-failure

# pygenでLua/C++バインディングコードとFontAwesomeアイコンテーブルを再生成する
autogen build_dir="build":
    [ -f {{build_dir}}/CMakeCache.txt ] || cmake -S . -B {{build_dir}}
    cmake --build {{build_dir}} --target pygen

# clang-formatで全ファイルをフォーマットする(書き換える)
format:
    python3 scripts/run_clang_format.py

# フォーマット・ビルド・テストをすべて確認する
check:
    python3 scripts/run_clang_format.py --check
    just build
    just test

build-docker:
  git submodule update --init --recursive
  docker build -t movutl scripts/
  docker run -it --rm -v $(pwd):/app movutl just build build-docker

