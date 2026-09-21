default:
    @just --list

# cmake configure(初回のみ) + ビルド。CMakeLists.txt変更時の再configureはcmakeのビルドルールが自動で行う
# --daw: MIDI/VST3(uapmd, vst3sdkのfetchとビルド)も含める。--no-daw: 外す。どちらも無ければ前回のconfigure設定を維持する(初回はOFF)
# 例: just build --daw
build *flags:
    #!/usr/bin/env bash
    set -euo pipefail
    build_dir=build
    daw=""
    for f in {{flags}}; do
      case "$f" in
        --daw) daw=ON ;;
        --no-daw) daw=OFF ;;
        *) echo "unknown flag: $f (--daw / --no-daw)" >&2; exit 1 ;;
      esac
    done
    if [ ! -f $build_dir/CMakeCache.txt ]; then
      # uapmdサブモジュールはDAW機能の時だけ取得する
      if [ "${daw:-OFF}" = ON ]; then
        git submodule update --init --recursive
      else
        git submodule status | awk '{print $2}' | grep -v '^ext/uapmd$' | xargs git submodule update --init --recursive --
      fi
      cmake -S . -B $build_dir -DMOVUTL_DAW=${daw:-OFF}
    elif [ -n "$daw" ] && ! grep -q "^MOVUTL_DAW:BOOL=$daw$" $build_dir/CMakeCache.txt; then
      [ "$daw" = ON ] && git submodule update --init --recursive -- ext/uapmd
      cmake -S . -B $build_dir -DMOVUTL_DAW=$daw
    fi
    cmake --build $build_dir -j${BUILD_JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu)}

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

# pygenでLua/C++バインディングコードとFontAwesomeアイコンテーブルを再生成し、続けてclang-formatで整形する
autogen build_dir="build":
    [ -f {{build_dir}}/CMakeCache.txt ] || cmake -S . -B {{build_dir}}
    cmake --build {{build_dir}} --target pygen
    python3 scripts/run_clang_format.py

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

