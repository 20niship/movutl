default:
    @just --list

# cmake configure + ビルド
build build_dir="build":
    git submodule update --init --recursive
    cmake -S . -B {{build_dir}}
    cmake --build {{build_dir}} -j${BUILD_JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu)}

# ビルドしてテストを実行する
test: build
    ctest --test-dir build --output-on-failure

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

