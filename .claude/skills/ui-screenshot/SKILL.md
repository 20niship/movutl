---
name: ui-screenshot
description: movutlのGUIを実際に起動してスクリーンショット(PNG)を撮り、UI変更が正しく動作・表示されているか確認する。UIを変更した後の動作確認、見た目の不具合調査、PRに貼る画像の取得のときに使う。
---

# movutl の UI スクリーンショットで動作確認する

画面収録権限が無い環境でも、アプリ内蔵の `movutl.export_screen_png(path)`(`app.hpp`。中身は `gui.hpp` の `capture_screen()`=Vulkan swapchainの直近描画結果のコピーをreadback)を Lua から呼んで、ImGui UI込みの画面を PNG に保存できる。外部の `screencapture` は使わない。

## 手順

1. **ビルド**(新規ファイルを追加した場合は先に `cmake -S . -B build`):
   ```
   cmake --build build -j8
   ```
2. **撮影スクリプトを実行**(必ず `build/` から。相対パスでアセットを解決するため)。GUIは常駐するので、スクリプトが `os.exit(0)` で終了する:
   ```
   cd build
   MU_CAPTURE_OUT=<出力ディレクトリ> \
   MU_CAPTURE_EXO=<開くexo/プロジェクト/メディア(省略可)> \
   MU_CAPTURE_SELECT=<選択するEntityの通し番号(省略時0)> \
   MU_CAPTURE_FRAME=<移動するフレーム(省略可)> \
   ./movutl_main ../examples/ui_capture.lua > <ログ> 2>&1 &
   sleep 20; pkill movutl_main   # 念のため。正常なら自分で終了している
   ```
   - シェルの `noclobber` が有効な環境では `>` が失敗するので `>|` を使う。
   - macOS には `timeout` コマンドが無い。`sleep` + `pkill` で代用する。
   - 出力は `01_initial_000000.png`(初期)、`02_select…`、`03_after_select…`(Entityを選択した状態=インスペクタの確認用)。
3. **画像を確認**: `Read` ツールで PNG を開いて目視する(2560x1440 を縮小表示する。座標は表示サイズ→元画像で 1.28 倍)。ログに `ui_capture: … true` が出ていることも確認する(`false` は撮影失敗)。
4. 問題があれば修正して 1 から繰り返す。

## `examples/ui_capture.lua` の仕組み
- `movutl.open_file(exo)` で読み込み、`movutl.goto_frame` で再生ヘッドを移動。
- `movutl.register_frame_hook(fn)`(`binding.cpp`。ウィンドウを作らず毎フレームfnを呼ぶ)で「N フレーム待つ → 操作(`act`) → 撮影」を順に実行する。撮影は表示済みバッファを読むので、操作の数フレーム後に行う(`wait` で調整)。
- Entity 選択は `movutl.select_entt_by_index(i)`(`app.cpp`)。派生クラス(`TextEntt`等)は LuaIntf 上で `Entity` に暗黙変換できないため、`select_entt` に直接渡せない。
- ステップを増やすには `steps` に `{ wait = フレーム数, name = "ファイル名", act = function() … end }` を足す。別のUI状態(ポップアップを開く、ソロを押す等)を撮りたいときは、`act` からコマンドや API を呼ぶ。

## 撮影時の注意
- **自動テストとしては使わない**(見た目の確認用)。結果は必ず目で見る。
- スクショは毎フレームswapchainからGPU内コピーしておいた画像を読むため、ウィンドウが隠れていても撮れるが、最小化中は描画自体が止まり更新されない。
- 起動直後はワーカーが描画キャッシュを作る途中でビューアが黒いことがある。`wait` を増やす。
- `mutest` を別のエージェント/端末と同時に走らせると `/tmp` の共有ファイルで競合して一時的に失敗することがある(GUI撮影とは無関係)。

## 確認するときの観点(例)
- ラベル・値が見切れていないか(インスペクタ、タイムラインのクリップ名、ステータスバー)。
- 選択状態が強調されているか、ギズモ枠が実描画と合っているか。
- ステータスバーの操作ログ・タイトルが更新されているか。
- 音量メーター/フェーダーがタイムライン右端に収まり、目盛りが他要素に重なっていないか。
