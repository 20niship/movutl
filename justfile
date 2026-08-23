default:
    @just --list

# cmake configure + ビルド
build:
    cmake -S . -B build
    cmake --build build -j

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
  docker exec -it --rm -v $(pwd):/movutl movutl just build

