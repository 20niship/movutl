# GPU(Vulkan)レンダラー導入計画

Issue#3 6.2「キャッシュRendererのGPU化(Vulkan)」の実装計画。本ドキュメントは計画のみで、実装は含まない。

## 1. 背景と目的

- 現状は `CPURenderer` のみ。合成(移動/回転/拡大/alpha/blend)は CPU の `Image::place` で行う。
- 目的:
  1. `Renderer` を継承した `VulkanRenderer` を作り、レイヤごとの合成・図形描画をGPUで行う(エフェクトはCPUのままで良い)。
  2. 色調補正/反転/並べて配置(tile)等の画像処理を Vulkan compute shader で行えるようにし、汎用シェーダ実行ユーティリティ(utility class/関数)を作ってエフェクトから使う。
  3. AviUtl2 の `filter2.h`(`exec_pixelshader_file` 等)と形状互換のAPIを提供する。
  4. Renderer を名前で切り替えられるようにする(レジストリ)。デフォルトは引き続き CPU。
- 参考: AviUtl2 SDK `ext/aviutl2_sdk_mirror/include/aviutl2_sdk/filter2.h`

## 2. 現状調査(要点)

### Renderer
- `movutl/render2d/renderer.hpp`: `Renderer` は `virtual bool render_frame(Composition*, int frame, Ref<Image>& out) = 0`。`CPURenderer` が実装し、非virtualの4引数版 `render_frame(comp, frame, out, transparent_bg)` に転送する(実装は `renderer.cpp:13-160`)。
- `CPURenderer` の直書き箇所は3つ:
  - `movutl/asset/composition.cpp:201` (`render_current_frame_main_thread`)
  - `movutl/render2d/render_worker.cpp:97` (プレビュー先読み/緊急要求。worker複数スレッド)
  - `movutl/gui/export_window.cpp:41` (エクスポート。`Renderer` インターフェース経由なのはここだけ)
- `CompoRefEntt::render` (`movutl/asset/compo_ref.cpp:22`) がネストcompositionを `transparent_bg=true` で描画。
- `FrameCache` (`movutl/render2d/frame_cache.hpp`) は composition ごとの `Ref<Image>`(CPU)キャッシュ。

### 合成ロジック(GPU化の対象)
- レイヤ合成ループ: `renderer.cpp:36-157`。背景塗り(24-32)、group/camera 親変換(64-91、`GroupXformScope` はthread-local)、シーンチェンジ(93-127、A/Bを別Imageに描画して画素ごとに混合)、上のオブジェクトでクリッピング(128-142、上レイヤのalphaをmaskに)。
- `Entity::composite` (`entity.cpp:189-203`) が `Placement` (`image.hpp:16-25`: xy/anchor/scale/aspect/rot_x,y,z/alpha/blend) を作り `Image::place` (`image.cpp:221`) を呼ぶ。
  - 回転なし/z回転のみ: `copyto` / `transform_to`。`rot_x/rot_y` ≠ 0: `drawquad`(透視変換)。
  - blend は `BlendType` 10種(Alpha, Add, Sub, Mul, Div, Screen, Overlay, Darken, Lighten, HardLight)で `blend_channel<B>` / `blend_pixel_t<B>` (`image.cpp:62-105`)。
- フレームバッファは `Image` (`movutl/asset/image.hpp`)。RGBA8 (`Vec4b`)。
- エンティティごとの `render(Composition*, Image* target, int frame)` は Image/Movie/Text/Shape/Framebuffer/CompoRef/CustomObject が実装済み。ソース画像生成はCPUで行われる。
- フィルタ: `movutl/plugin/filter.hpp` の `FilterPluginTable`(関数ポインタ構造体)。組込みは `movutl/plugin/default/register_default_plugins.cpp` に登録。`f_invert`(`image_tone_filter.cpp:38`)、`f_color_correction`(`image_color_filter.cpp:33`)。**tile(並べて配置)は未実装**。`Entity::render_filters` (`entity.cpp:284-315`)。

### 3D
- `Mesh` 系・`EntityType_3DModel/Primitive` は存在するが `Entity::CreateEntity` が生成せずCPU側でも未接続。`light.hpp` は空。**本計画では3Dは対象外**。

### GUI/ビルド
- GUIは ImGui + GLFW + OpenGL3(GLEW)。GL利用箇所は `movutl/graphics/GLTexture.*`、`movutl/gui/viewer.hpp`(`GLTexture tex`)、`gui/window.cpp`、`gui/gui.cpp` と小さい。`ext/imgui/backends/imgui_impl_vulkan.*` は同梱済み。
- `movutl/CMakeLists.txt` に `find_package(Vulkan QUIET)` があるが未使用。
- テストは `tests/*.cpp`(doctest、globなのでファイル追加後にcmake再実行)。`mutest` を ctest が実行。`tests/renderer_test.cpp` が `CPURenderer` を使う典型例。
- CIは ubuntu-latest で `just check`。GPUなし。
- 開発機(macOS)には homebrew で `molten-vk` `vulkan-loader` `vulkan-headers` `glslang` `spirv-tools` `glfw` 導入済み。
- AviUtl2 `filter2.h` は `ID3D11*`, `LPCWSTR`, `BYTE` 等 Windows/D3D11型に依存し、macOS/Linuxでは直接includeできない。`exec_pixelshader_file` 等は `.cso`(DXBC)を要求する。

## 3. 決定事項

| 項目 | 決定 |
|---|---|
| 対象OS | macOS(MoltenVK) + Linux + Windows。CIはLinuxで lavapipe(CPU Vulkan)を使いヘッドレステスト |
| GUI | OpenGLを廃止し ImGui Vulkanバックエンドに一本化。GPU結果はゼロコピーで表示 |
| 出力 | mp4等の出力プラグインへはCPU `Image` にreadbackして渡す |
| シェーダ | 実行時にGLSLをglslangでSPIR-Vへコンパイル(組込みシェーダも同経路) |
| AviUtl2互換 | API形状互換 + SPIR-V/GLSL。`.cso`は非対応(`false`を返しログ) |
| 3D | 対象外 |
| Renderer切替 | レジストリ(名前→ファクトリ) + 起動引数/環境変数 + 設定画面。設定は `movutl_cnf.json`(`Config`)に保存し、次回起動時に適用 |
| デフォルト | CPURenderer のまま |

## 4. 全体設計

### 4.1 Rendererレジストリ
- `movutl/render2d/renderer_registry.hpp/.cpp`
  - `using RendererFactory = std::function<std::unique_ptr<Renderer>()>;`
  - `register_renderer(name, factory)` / `create_renderer(name)` / `renderer_names()`
  - 組込み `"cpu"`, `"vulkan"`(Vulkan初期化に失敗したら `"cpu"` にフォールバックしログ)
- `Renderer::render_frame(comp, frame, out, transparent_bg)` の4引数版を virtual にし、3箇所の直書きを `create_renderer(...)` に置換する。
- 選択の優先順位: 起動引数 `--renderer=<name>` > 環境変数 `MOVUTL_RENDERER` > `Config::renderer`(`movutl_cnf.json`)> `"cpu"`。
- worker毎にRendererインスタンスを作るか共有かは Phase 4 で決める(Vulkanは共有 `VkContext` + スレッド毎command pool)。

### 4.2 Vulkan基盤 `movutl/vulkan/`
- `vk_context`: instance/physical device選択/device/queue/VMA相当の簡易アロケータ。surface無しのヘッドレスも可。GUIとレンダラーで同一 `VkDevice` を共有する。queue submitはmutexで直列化、command poolはスレッド毎。
- `vk_image`: RGBA8(必要ならRGBA16F)GPU画像。view/sampler/layout遷移管理、staging bufferによる `upload(const Image&)` / `readback(Image&)`。
- `shader_util`: GLSL(comp/vert/frag)→SPIR-V(glslang)。ソースhashをキーにSPIR-V、`VkShaderModule`、pipeline、descriptor set layoutをキャッシュ。エラーは行番号付き文字列で返す(例外/クラッシュにしない)。
- `gpu_compute`(汎用ユーティリティ。エフェクトはこれだけ使う):
  - `run_compute(glsl, inputs, outputs, params(push constant/UBO), groups)`
  - `run_fullscreen(frag_glsl, target, inputs, params, blend)`
  - 入出力に `vk_image` または `Image`(内部でupload/readback)を受ける。
- `vulkan_renderer`: `Renderer` 継承。

### 4.3 合成アルゴリズム
1. `comp->get_layered_entities()` をCPU版と同順に走査(mutex/visible/`apply_animated_props`は既存と同じ)。
2. エンティティのソース画像をCPUで生成し(`render` + CPUフィルタ)、GPUへ upload。キー(entity id + frame + dirty/hash)でテクスチャキャッシュ。
3. `Entity::composite` の `Image::place` 呼出しを sink 抽象(`PlaceSink`: CPU=`Image::place`、GPU=quad描画)に差し替える。
4. `Placement` を textured quad の頂点/行列へ変換(anchor/aspect/scale/rot_z は2D行列、rot_x/rot_y は `drawquad` と同じ透視)。
5. blend: Alpha/Add は fixed-function blend。その他は fragment shader で dst を読む(dstをコピーする ping-pong)。式は `image.cpp:62-105` と同じにして丸めを揃える。
6. シーンチェンジ: A/Bを別オフスクリーンに描画し遷移パスで混合。クリッピング: 上レイヤのalphaをmask textureとして使う。group/camera の親変換は `GroupXformScope` の結果を行列に反映。
7. 結果 `vk_image` を返す。readbackが必要な場合のみ `Ref<Image>` へ変換。

### 4.4 表示(ゼロコピー)
- viewer は `ImGui_ImplVulkan_AddTexture(sampler, view, layout)` で描画された `vk_image` を直接表示。
- CPUレンダラ経路の結果は `vk_image` へuploadして表示(`GLTexture` の置換)。
- `FrameCache` はCPU Imageを保持し続けるが、GPU経路ではGPU画像を保持するキャッシュ(サイズ上限あり)を追加。エクスポート/CPUを必要とする箇所のみ readback。

### 4.5 AviUtl2 filter2互換
- `filter2.h` は Windows/D3D11型依存のため直接includeせず、移植可能な定義を `movutl/plugin/gpu/filter2_compat.hpp` に置く。
  - そのまま使う: `VERTEX_COLOR`, `VERTEX_COLOR_NORM`, `VERTEX_TEXTURE`, `VERTEX_TEXTURE_NORM`, `VERTEX_TYPE`, `BLEND_MODE`
  - 置換: `LPCWSTR`→`const wchar_t*`(内部でUTF-8化)、`ID3D11BlendState*`/`ID3D11SamplerState*`→独自enum/ポインタ、`ID3D11Texture2D*`は提供しない
- 提供する関数(同名・同じ引数の並び): `draw_image`, `draw_poly`, `draw_poly_to_resource`, `set_blend_mode`, `exec_pixelshader_file/_data`, `exec_computeshader_file/_data`, `get_image_data`, `set_image_data`
- リソース名: `"object"`(nullptr含む), `"resource:xxxx"`, `"tempbuffer"`, `"framebuffer"`, `"cache:xxxx"`, `"image:xxxx"`, `"random"`(256x256, R32F 乱数)
- シェーダ入力: `.spv` または GLSL(`.frag`/`.comp`)。`.cso`(DXBC)は `false` を返しログ。ピクセルシェーダの座標/UV規約(`SV_Position`,`TEXCOORD`)はGLSLの `gl_FragCoord` / `layout(location=0) in vec2 uv` に対応させる規約をドキュメント化する。

### 4.6 ビルド/CI
- `movutl/CMakeLists.txt`: `find_package(Vulkan REQUIRED)`、glslang(`glslang`,`SPIRV`)、`imgui_impl_vulkan.cpp` 追加、`imgui_impl_opengl3.cpp` / GLEW / `RENDERER=WITH_OPENGL` 削除。
- justfile: 依存確認の追加が必要なら追記。CI(`.github/workflows/ci.yml`): apt に `libvulkan-dev glslang-dev glslang-tools libglslang-dev spirv-tools mesa-vulkan-drivers`(lavapipe)を追加。
- テストは Vulkan デバイス/`VkContext` 生成に失敗する環境では SKIP(警告出力)する。

## 5. フェーズ計画

各フェーズはPRを分けられる粒度。ゴールと受け入れテストを明記する。

### Phase 1: Rendererレジストリ + Config切替(挙動不変)
**ゴール**: `CPURenderer` 直書きを排除し、名前でRendererを切り替えられる。

**タスク**
1. `renderer_registry` 新設、`"cpu"` 登録。
2. `Renderer` の4引数版を virtual 化。
3. 3箇所(`composition.cpp:201`, `render_worker.cpp:97`, `export_window.cpp:41`)を `create_renderer` に置換。
4. `Config` に `renderer`(文字列、default `"cpu"`)を追加し `movutl_cnf.json` に保存/読込。
5. `--renderer=` 引数と `MOVUTL_RENDERER` 環境変数の解決処理(優先順は 4.1)。
6. 設定画面にコンボボックス(現在値と、再起動で適用される旨の注記)。
7. `just autogen`(`MPROPERTY` / 公開関数を変更する場合)。

**テスト**
- `create_renderer("cpu")` が非null。
- 未登録名はフォールバックして `"cpu"` が返り、ログが出る。
- 既存 `renderer_test` / `blend_test` / `scene_change_test` / `camera_test` が全て通る。
- `Config` を保存→読込して `renderer` が保持される。
- `MOVUTL_RENDERER=xxx` が Config を、`--renderer=` が環境変数を上書きする(解決関数の単体テスト)。

### Phase 2: Vulkanコンテキスト + GUIのVulkan化(GL廃止)
**ゴール**: OpenGLなしでGUIが起動し、CPU描画結果がVulkanテクスチャとして表示される。ヘッドレスで `VkContext` が作れる。

**タスク**
1. `vk_context`(ヘッドレス/surface付き両対応、MoltenVKのportability列挙拡張に対応)。
2. `vk_image`(upload/readback)。
3. `gui/window.cpp` を GLFW(`GLFW_CLIENT_API=NO_API`) + swapchain + `imgui_impl_vulkan` に変更。リサイズ時のswapchain再生成。
4. `GLTexture` → `VkTexture` に置換、`viewer` を対応。
5. GLEW/OpenGL関連のCMake・コード除去。
6. CIにVulkan依存を追加。

**テスト**
- ヘッドレス `VkContext` が生成できる(デバイス無しはSKIP)。
- `vk_image` に乱数 `Image`(256x128)を upload→readback して完全一致。
- サイズ1x1 / 非2冪 / 大サイズ(4096x2160)でも upload→readback が一致。
- `just run` を起動し ui-screenshot skill でスクショ確認(既存UIが崩れていない)。
- CI(lavapipe)で `mutest` が通る。

### Phase 3: 汎用シェーダユーティリティ
**ゴール**: GLSLを渡すだけで compute/fragment を実行でき、エフェクトから1関数呼びで使える。

**タスク**
1. `shader_util`: glslang初期化/終了、GLSL→SPIR-V、エラー整形、ソースhashキャッシュ。
2. pipeline / descriptor set layout のキャッシュ(入出力数・push constantサイズから自動生成)。
3. `gpu_compute::run_compute` / `run_fullscreen`。
4. `Image` を直接受ける簡易ラッパ(upload→実行→readback。エフェクト向け)。
5. 乱数バッファ(256x256, R32F)のユーティリティ。

**テスト**
- compute で `out = vec4(255-r,255-g,255-b,a)` → CPU期待値と一致。
- 不正GLSLがエラー文字列(行番号を含む)を返し、クラッシュ/例外にならない。
- 同一ソース2回実行でコンパイル回数が1(キャッシュヒットのカウンタで確認)。
- push constant / UBO の値が反映される。複数入力画像・複数出力が動作する。
- 乱数バッファは値域が0〜1で、実行間で同じ内容(シード固定)。

### Phase 4: VulkanRenderer 2D合成
**ゴール**: 全レイヤ合成をGPUで行い、`CPURenderer` と一致(各ch差 ≤2)。

**タスク**
1. `VulkanRenderer`(`"vulkan"` としてレジストリ登録)、`VkContext` 共有、スレッド毎command pool。
2. `PlaceSink` 抽象を導入し `Entity::composite` を差し替え可能に。CPUは既存動作のまま。
3. textured quad パイプライン(移動/回転z/拡大/anchor/aspect/alpha)。
4. blend 10種(fixed-function + shader dst読み)。
5. ソースImageのテクスチャキャッシュ(key: entity id + frame + hash)。
6. 背景色 / `transparent_bg=true`。
7. 結果を `Ref<Image>` にreadbackして `render_frame` のインターフェースを満たす。

**テスト**(同一シーンをCPU/GPUで描画し差分≤2)
- Shape / Image / Text / Movie 各単体。
- 複数レイヤの重なり。
- alpha 0 / 0.5 / 1。blend 全10種。
- 移動、z回転、拡大、anchor、aspect。
- 背景色あり/なし、`transparent_bg=true`(nested composition)。
- 境界: 空composition、範囲外に完全にはみ出したレイヤ、1x1 composition。
- worker複数スレッドから同時に `render_frame` を呼んでも結果が同一(データ競合なし)。

### Phase 5: 高度な合成 + ゼロコピー表示
**ゴール**: AviUtl仕様の高度合成(`renderer.cpp:64-142`)をGPUで再現し、プレビューはGPU画像を直接表示。

**タスク**
1. rot_x/rot_y の透視(`drawquad` 相当)。
2. group/camera の親変換(`GroupXform`)を行列に反映。
3. シーンチェンジ(A/Bオフスクリーン + 遷移パス、`SceneChangeWeight`)。
4. 「上のオブジェクトでクリッピング」(mask)。
5. viewer がGPU画像を `ImGui_ImplVulkan_AddTexture` で直接表示。
6. GPUフレームキャッシュ(メモリ上限付き)。エクスポート経路でのみ readback。

**テスト**
- `scene_change_test` / `camera_test` / クリッピングの既存シーンをCPU/GPU比較 ≤2。
- rot_x/rot_y 各種(±30°, 60°, 89°付近)をCPU/GPU比較。
- プレビュー中に readback が発生しない(readbackカウンタ=0)。
- エクスポートで readback 済み `Image` が出力プラグインへ渡り、ffmpeg出力の先頭フレーム画素が期待値と一致。
- GUIスクショで通常/回転/透視表示を確認。

### Phase 6: GPUエフェクト(compute)
**ゴール**: 主要エフェクトを `gpu_compute` で実装し、CPU版と結果が一致。

**タスク**
1. 反転(`f_invert`)、色調補正(`f_color_correction`)、グレースケール等の単純系をGPU版に。
2. 新規エフェクト「並べて配置(tile)」(横/縦の個数、間隔、端数処理)をGPUで実装(CPU版もフォールバックとして用意)。
3. Vulkanレンダラーではソースが既にGPU上にあるためupload/readbackを挟まないGPUエフェクト経路を用意。CPUレンダラー(または非対応フィルタ)ではCPU版を使う。
4. エフェクト側は `gpu_compute` の利用だけで書く(Vulkan APIを直接呼ばない)。

**テスト**
- 反転 / 色調補正 / tile をCPU版と比較(≤1〜2)。
- tile: 2x2, 3x1, 割り切れないサイズ、個数1。
- Vulkanレンダラー上でGPU版、CPUレンダラー上でCPU版が使われる(選択ログ/カウンタで確認)。
- GPUエフェクト後に続けてCPUフィルタを適用してもレイアウトが崩れない。

### Phase 7: AviUtl2 filter2互換API
**ゴール**: `FILTER_PROC_VIDEO` と同形状の関数でGPU描画/シェーダ実行ができる。

**タスク**
1. `movutl/plugin/gpu/filter2_compat.hpp`(4.5)。
2. リソース管理: `object` `resource:xxxx` `tempbuffer` `framebuffer` `cache:xxxx` `image:xxxx` `random` の名前解決とライフサイクル。
3. `exec_pixelshader_*` / `exec_computeshader_*`(GLSL/`.spv`)、`draw_image` / `draw_poly` / `draw_poly_to_resource` / `set_blend_mode` / `get_image_data` / `set_image_data`。
4. 互換ドキュメント(座標系・UV・定数バッファ b0 → UBO/push constant の対応)を `docs/` に追記。
5. サンプルGLSLエフェクトを `examples/` に追加。

**テスト**
- `exec_pixelshader_file`(GLSL frag)で `object` に単色出力。
- `exec_computeshader_file` で `object` を反転。
- `draw_poly`: `TRIANGLE_COLOR` / `QUAD_TEXTURE` の頂点リストで数画素を検証。
- 不正なリソース名・頂点数(3の倍数でない等)で `false`。
- `.cso` 指定で `false` + ログ。
- `set_blend_mode` の各モードが `draw_image` に反映される。

## 6. リスク・未確定事項

- MoltenVK でGUIとレンダラーが同一 `VkDevice` を共有する構成、swapchain再生成時の同期。
- 4引数 `render_frame` のvirtual化と、`FrameCache` を前提にしたプレビュー経路(`viewer.cpp:81-87`、`app.cpp:214`)への影響。
- Windowsビルドは現CIに無く未検証。Vulkan SDK/glslangの取得方法を別途決める。
- glslangのライブラリ名/CMakeターゲットがHomebrew/apt/vcpkgで異なる。
- CPUとGPUの丸め差。直近でImage合成の丸め誤差を修正済みのため、許容値は±2で開始し縮められれば縮める。
- ソース画像のCPU生成+uploadがボトルネックになる場合、Text/Shape/Movieデコードのフレームキャッシュ強化が必要(本計画の範囲外)。
- 3D(Mesh/Light)は対象外。将来対応する場合はデプスバッファを持つ別パスを追加する。

## 7. 検証手順(全フェーズ共通)

1. `just build`(Vulkan必須になった後も同コマンド)
2. `just test`(mutest全通過、Vulkan無し環境は該当テストSKIP)
3. UI変更時は `just run` + ui-screenshot skill でスクショ確認
4. `just autogen` → `git status` で生成物/整形差分を確認 → `just check`
