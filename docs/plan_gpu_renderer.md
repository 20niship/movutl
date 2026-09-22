# GPU(Vulkan)レンダラー導入計画(残タスク)

Issue#3 6.2「キャッシュRendererのGPU化(Vulkan)」の実装計画。Phase1〜4は完了、Phase5/6は一部完了・残タスクあり、Phase7は未着手。

## 完了分(サマリ)

| Phase | 内容 | PR | 主な成果物 |
|---|---|---|---|
| 1 | Rendererレジストリ + Config切替 | #50 | `movutl/render2d/renderer_registry.{hpp,cpp}`、`Config::renderer` |
| 2 | Vulkanコンテキスト + GUIのVulkan化(GL廃止) | #51 | `movutl/vulkan/vk_context.*`, `vk_image.*`、`movutl/graphics/VkTexture.*`、`gui/window.cpp`(GLFW+swapchain+imgui_impl_vulkan) |
| 3 | 汎用シェーダユーティリティ | #51 | `movutl/vulkan/shader_util.*`(glslang実行時コンパイル)、`gpu_compute.*`(`run_compute`/`run_compute_image`) |
| 4 | VulkanRenderer 2D合成 | #52 | `movutl/vulkan/vulkan_renderer.*`、`movutl/render2d/composite_ops.*`、`GpuCompositeSink`(entity.hpp/cpp) |

以降の実装は上記を再利用すること。特に `run_compute_image(glsl, inputs, output, params, err)` (movutl/vulkan/gpu_compute.hpp) は「Image一枚渡してGLSLで加工して受け取る」用途に一番手軽。新規GPU処理を書く前にまずこれで足りないか検討する。

## Phase 5: 高度な合成 + ゼロコピー表示(残タスク: ゼロコピーのみ)

**完了済み**: rot_x/rot_y透視・group/camera親変換は Phase4 の `VulkanRenderer::build_xform` / `GroupXformScope` 連携で実装済み。シーンチェンジ・クリッピングは `composite_ops.hpp` 経由のCPUブリッジで動作確認済み(`tests/vulkan_renderer_test.cpp`)。追加実装は不要、変更する場合は既存テストの±2px許容(`images_close`の`max_bad_pixels`)を壊さないよう回転四隅の数式(`vulkan_renderer.cpp`の`build_xform`)をCPU版(`image.cpp`の`drawquad`)と照合すること。

**残タスク: プレビューのゼロコピー表示 → 調査の結果、当初案では不十分と判明。未着手のまま設計を更新**

以前のこの節は「`VulkanRenderer`に`last_gpu_result()`のようなgetterを足し、viewerがそれを直接表示する」という小さい差分案を書いていたが、実際のプレビュー経路を調べた結果、この案では主要経路をカバーできないことが分かった。

- **実際のプレビュー経路**(`movutl/gui/viewer.cpp:81`): `comp->cache.get_nearest(comp->frame, &img)` で `FrameCache`(`movutl/render2d/frame_cache.hpp`、`Ref<Image>`のみを保持)から読む。`Composition::render_current_frame_main_thread()`(getterを足す対象だった関数)は「初回フレームでキャッシュが1枚も無い時の同期フォールバック」(`viewer.cpp:87`)専用で、通常再生中には呼ばれない。
- **実描画は `RenderWorkerPool`(`movutl/render2d/render_worker.cpp`)が担う**: 最大4本のバックグラウンドスレッドが起動時に`create_active_renderer()`で**スレッド毎に1つ**`Renderer`インスタンスを保持し続け(`worker_loop`)、ジョブが来るたびに`render_frame(job.comp, job.frame, out)`→`Ref<Image>`を`comp->cache`へ`insert`する。つまりGPU画像が実際に作られるのはGUIスレッドではなくワーカースレッド上であり、getterで拾えるのは「そのワーカーの`VulkanRenderer`が今保持している最新の1枚」でしかない。
- **`FrameCache`はCPU向けの設計で、GPU画像をそのまま流用できない**: `Config::cache_frames`の既定は1024フレーム。CPUの`Ref<Image>`(RAM)なら1024枚保持は現実的だが、GPU VRAM上の`GpuImage`を同じ枚数保持するのはフルHD相当で数GB〜のVRAMを消費し非現実的。GPU用には全く別の(枚数を絞った)キャッシュ設計が要る。
- **スレッド間のライフタイム管理が必要**: ワーカースレッドの`VulkanRenderer`は`ensure_target`で`gpu_out_`(`GpuImage`)を毎フレーム使い回す。GUIスレッドがこの`GpuImage`をそのまま表示に使うには、「GUIスレッドが表示に使っている間はワーカーがその`GpuImage`を上書きしない」保証(ダブルバッファや参照カウント、Vulkanフェンス待ち)が要る。生ポインタをそのまま`last_gpu_result()`で渡すのは、表示中に次フレームの合成で上書きされる競合状態(データ破壊/フリッカー)を招くため実装しない。
- **結論**: ゼロコピー本実装には、(a) `FrameCache`とは別にGPU画像用の小容量キャッシュ(せいぜい表示中の数フレーム分)を設計する、(b) `RenderWorkerPool`のワーカースレッドからGUIスレッドへ`GpuImage`の所有権/生存期間を安全に受け渡す仕組みを作る、(c) `VkTexture`に`GpuImage`を直接受け取るオーバーロードを足す、の3点が必要で、これは`Composition`/`RenderWorkerPool`双方に踏み込む設計変更になる。「小さい差分」の範囲を超えるため、今回は見送り、上記3点を次に着手する際の入口として残す。
- **現状維持**: `gpu_readback_count()`(`movutl/vulkan/vk_image.hpp`)による計測は既に入っており、1フレーム1回のreadbackが発生していることは実測済みのまま。プレビューは引き続きCPU経由(`FrameCache`→`VkTexture::set(Ref<Image>)`のCPU再アップロード)で動作する(表示自体は正しく動く。今回はパフォーマンス最適化が未着手というだけ)。

## Phase 6: GPUエフェクト(compute)(完了)

反転・色調補正・新規「並べて配置(tile)」はGPU版(`movutl/plugin/gpu/gpu_effects.{hpp,cpp}`)とCPU版(`movutl/plugin/default/image_tile_filter.{hpp,cpp}`、既存`f_invert`/`f_color_correction`)が一致(tileは完全一致、反転・brightness/contrastは差1以内)。テストは`tests/gpu_effect_test.cpp`。

**hue/saturationのCPU一致(解決)**: `gpu_effects.cpp`の`rgb2hsvFullByte`/`hsv2rgbFullByte`にOpenCVの`COLOR_RGB2HSV_FULL`/`HSV2RGB_FULL`(8u)と同じ整数シフトテーブル式(`hsv_shift=12`)・sector展開式を移植した。RGB→HSVはほぼ完全一致(ランダム画素6000chで一致率99.7%、残りも差1)。HSV→RGB復元は近似が残り各ch差が最大7程度出るため、テスト(`tests/gpu_effect_test.cpp`)は許容差8で確認している。以前の連続角度近似(差最大223)から大幅に改善した。

**GPU/CPU自動切替(完了)**: `FilterPluginTable`(`movutl/plugin/filter.hpp`)末尾に`fn_proc_gpu`(任意、nullptr可)を追加し、`Entity::render_filters`(`entity.cpp`)で`active_renderer_name()=="vulkan"`かつ非nullなら`fn_proc_gpu`を呼ぶよう分岐した。`f_invert`/`f_color_correction`/`f_tile`の3つだけ`register_default_plugins.cpp`でラッパを配線済み(他フィルタは`fn_proc_gpu=nullptr`のまま、過剰実装しない)。エンドツーエンドの自動切替確認は`tests/gpu_effect_autoswitch_test.cpp`(`Image::render()`をCPU/vulkan両方のアクティブレンダラーで呼び、同じ結果になることを確認)。

**新規実装時の注意**: `FilterPluginTable`の全既存初期化は6箇所以上が位置指定の集成体初期化(`{GUID(...), FilterDefault, ..., nullptr, nullptr}`)なので、構造体にフィールドを追加する場合は必ず**末尾**に追加すること(途中に挿入すると全既存フィルタの初期化がずれて壊れる)。`fn_proc_gpu`の値も、位置指定初期化に混ぜようとすると`reserve[2]`配列や`props`/`defaults`の型が合わずコンパイルエラーになるため、`register_default_plugins.cpp`で定義後に`f_xxx.fn_proc_gpu = ...;`と代入する方式にした。

## Phase 7: AviUtl2 filter2互換API(未着手)

**ゴール**: `FILTER_PROC_VIDEO`(`ext/aviutl2_sdk_mirror/include/aviutl2_sdk/filter2.h`)と同形状の関数でGPU描画/シェーダ実行ができる。

**実装方針**(Phase4-6で分かったことを反映):
- `filter2.h`は`ID3D11*`/`LPCWSTR`/`BYTE`等Windows/D3D11型に依存し直接includeできない。移植可能な定義を`movutl/plugin/gpu/filter2_compat.hpp`に新規に書く。
  - そのまま使えるもの: `VERTEX_COLOR`, `VERTEX_COLOR_NORM`, `VERTEX_TEXTURE`, `VERTEX_TEXTURE_NORM`, `VERTEX_TYPE`, `BLEND_MODE`(値だけコピーすれば良い、`filter2.h`をincludeしない)
  - 置換: `LPCWSTR`→`const wchar_t*`(内部でUTF-8化して扱う)、`ID3D11BlendState*`/`ID3D11SamplerState*`→独自enum、`ID3D11Texture2D*`は提供しない
- **shader実行は`movutl/vulkan/gpu_compute.hpp`の`run_compute`/`run_compute_image`をそのまま使う**。`exec_computeshader_file/_data`は`run_compute`に、`exec_pixelshader_file/_data`はfragment相当のcompute(フルスクリーン1スレッド/pixel)に読み替えて`run_compute`で実装できる(Phase3で`run_fullscreen`は未実装のまま終わったが、`run_compute`だけで十分代替できることがPhase4/6で実証済み。無理に`run_fullscreen`を別途作らなくてよい)。
- シェーダは`.spv`または生GLSL文字列。`compile_glsl`(`movutl/vulkan/shader_util.hpp`)がキャッシュ込みで使える。`.cso`(DXBC)は非対応、`false`を返しログ。
- リソース名解決(`"object"` `"resource:xxxx"` `"tempbuffer"` `"framebuffer"` `"cache:xxxx"` `"image:xxxx"` `"random"`)は名前→`GpuImage*`のマップを持つ小さなレジストリで良い。`"random"`は`gpu_compute.hpp`の`make_random_image(seed)`をそのまま使える(256x256 R32F)。
- `draw_image`/`draw_poly`/`draw_poly_to_resource`は、Phase4の`VulkanRenderer`が持つtextured quad描画(`vulkan_renderer.cpp`の`kCompositeGlsl`とほぼ同じ構造の合成compute shader)を土台に、頂点リスト(`VERTEX_COLOR`/`VERTEX_TEXTURE`等)を受け取れるよう一般化したcompute/描画関数を書く。`vulkan_renderer.cpp`の`build_xform`(dst→src逆変換の考え方)とpush constant設計をそのまま参考にできる。
- `get_image_data`/`set_image_data`は`GpuImage::readback`/`upload`(`vk_image.hpp`)をラップするだけで良い。

**タスク**
1. `movutl/plugin/gpu/filter2_compat.hpp`(上記の型定義)
2. リソース名解決とライフサイクル管理(名前→`GpuImage`のマップ、`"object"`は呼び出し中のフィルタ対象画像に固定)
3. `exec_pixelshader_*` / `exec_computeshader_*`(`run_compute`ベース)
4. `draw_image` / `draw_poly` / `draw_poly_to_resource` / `set_blend_mode`(`vulkan_renderer.cpp`のquad描画を一般化)
5. `get_image_data` / `set_image_data`(`GpuImage::readback`/`upload`のラップ)
6. 互換ドキュメント(座標系・UV・定数バッファb0→push constant/UBOの対応)を`docs/`に追記
7. サンプルGLSLエフェクトを`examples/`に追加

**テスト**
- `exec_pixelshader_file`(GLSL、`.spv`化せず生ソースでも可)で`object`に単色出力
- `exec_computeshader_file`で`object`を反転(Phase6の`gpu_invert`と同じ結果になることを確認できると尚良い)
- `draw_poly`: `TRIANGLE_COLOR` / `QUAD_TEXTURE`の頂点リストで数画素を検証
- 不正なリソース名・頂点数(3の倍数でない等)で`false`
- `.cso`指定で`false` + ログ
- `set_blend_mode`の各モードが`draw_image`に反映される(Phase4の`kCompositeGlsl`のblend_channelと同じ結果になるはず)

## リスク・未確定事項(継続分)

- Windowsビルドは現CIに無く未検証。Vulkan SDK/glslangの取得方法を別途決める。
- CIのLinux/lavapipe実行結果は要確認(Phase2-4のPRで初導入。lavapipeで実際にVulkanデバイスとして認識されているかログで確認すること。判別できない場合は`MESSAGE("SKIP: ...")`をCI上で`--success`付きで出す等、可視化を検討)。
- ソース画像のCPU生成+uploadがボトルネックになる場合、Text/Shape/Movieデコードのフレームキャッシュ強化や、Phase4で見送ったテクスチャキャッシュ(entity+frame+dirtyキー)の実装が必要(本計画の範囲外のまま)。
- 3D(Mesh/Light)は引き続き対象外。

## 検証手順(共通)

1. `just build`
2. `just test`(mutestは`build/`から実行すること。repo直下から実行するとアセット相対パスが解決できず失敗して見える)
3. UI変更時は `.claude/skills/ui-screenshot/SKILL.md` の手順でGUIを実起動しスクショ確認。`MOVUTL_RENDERER=vulkan`でも同様に確認する
4. `just autogen` → `git status` で生成物/整形差分を確認 → `just check`
