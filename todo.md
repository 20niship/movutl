# AviUtl互換 未実装機能リスト (todo)

> 座標系・基点・文字揃え・グループ制御の調査結果と統一仕様案は `docs/座標系と基点の仕様.md` を参照。Movie以外のEntityは位置の原点が左上で、exo取り込み時に位置がずれる問題あり。

調査元: `test.exo`(melchior_202609) / `movutl/plugin/aviutl_script/aviutl_obj_binding.cpp` / `movutl/command/exo/exo_import.cpp` / `movutl/asset/*.hpp`
AviUtl側の仕様は既知のobj.*拡張変数仕様に基づく(SDK mirrorにはスクリプト仕様が無いため、実装時に実機/ドキュメントで再確認すること)。

## 0. 方針

- **Phase 1(まず着手)**: AviUtl拡張変数の核 `obj` オブジェクトの差分を埋める(第3章)
- Phase 2: 基点(中心)・回転軸・縦横比などEntity共通の描画変換(第1章)
- Phase 3: テキストの揃え位置ほかexoテキスト属性(第2章)
- Phase 4: exoインポートの取りこぼし解消(第4章)

## 1. Entity共通の描画変換(基点まわり)

exoの`標準描画`/`拡張描画`にあるが、movutlのEntityに無い/使われていない項目。

| 項目 | AviUtl | movutl現状 | 対応 |
|---|---|---|---|
| 基点(中心X/Y/Z) `cx,cy,cz` | 回転・拡大の基点をオブジェクト中心からずらせる | `Entity::anchor_`(Vec2)はあるが**renderで未使用**。Movie/Image/Shape/Textは常に中心基点 | anchor_をVec3化しrender時に回転・拡大の基点へ反映。`obj.cx/cy/cz`と対応 |
| X/Y/Z軸回転 `rx,ry,rz` | 3軸回転 | Z回転(rotation)のみ | rx/ry追加(2D合成では射影変形。obj.drawpolyと共通化) |
| 縦横比 `aspect` | -100〜100 | 無し。Textのみscale_x/yあり | 共通のscale_x/scale_y相当へ統一 |
| 拡大率の単位 | % | Movie/Shape=%、Image/Text=0-1と**不統一** | 単位・回転(deg/rad)・alpha(0-255/0-1)をEntity間で統一(または変換層を明記) |
| Z座標 | 有り | Vec3のposはあるが2D合成で未使用の可能性 | 要確認 |
| 合成モード | blend=0..N | `Entity::blend_`あり(Blend_Alpha..HardLight) | exoの`blend`番号→BlendType対応表が未実装(第4章) |
| クリッピング/カメラ制御対象 | `camera=0`等 | `clipping_up_`, `camera_ctrl_`あり | exoインポートで未反映 |

## 2. テキストオブジェクト(exo `テキスト`)

test.exoのテキスト21個が持つキーのうちTextEntt(`asset/text.hpp`)が未対応のもの。

- [ ] **揃え位置 `align`**(左/中央/右 × 上/中/下 = 9通り、縦書き系含む) ← 要望
- [ ] **サイズ `サイズ`**(現状`re_render_image`はフォントサイズ16固定でexoのサイズを無視)
- [ ] 文字装飾 `type`(0:標準 1:影付き 2:影付き薄 3:縁取り 4:縁取り細) と装飾色 `color2`(現状はborder_width_のみ)
- [ ] 太字 `B` / 斜体 `I`
- [ ] 字間 `spacing_x` / 行間 `spacing_y`
- [ ] 等間隔 `monospace`
- [ ] 表示速度 `表示速度`(タイプライター表示)
- [ ] 文字毎に個別オブジェクト(`separate`プロパティはあるが挙動未実装の疑い→要確認)
- [ ] 移動座標上に表示する / 自動スクロール
- [ ] `autoadjust`(自動的に折り返し) / `soft`(ソフト表示) / `precision`(精度)
- [ ] 制御文字(`<b>`,`<#RRGGBB>`,`<s40>`,`<p+x,y>`等のインライン装飾)

## 3. obj(拡張変数のコアオブジェクト)の差分 ← **Phase 1**

現状: `plugin/aviutl_script/aviutl_obj_binding.cpp`の`setup_obj_table`で以下のみ提供。

**実装済み**
- 変数: `ox oy oz rx ry rz zoom alpha cx cy frame totalframe time layer framerate track0-3 check0-3`
- 関数: `getpixeldata putpixeldata getpixel(引数なし=w,h) getinfo effect draw drawpoly line(独自) copybuffer setoption(drawtarget) load(tempbuffer/obj) `
- no-op: `getoption setanchor setfont`
- グローバル: `RGB`

**未実装 / 不正確(要対応)**

変数:
- [ ] `obj.cz` `obj.aspect`(未定義。cx/cyもdraw/暗黙drawに**反映されていない**)
- [ ] `obj.x obj.y obj.z`(オブジェクトの基準座標・読み取り専用。現状0固定でEntityのposを渡していない)
- [ ] `obj.w obj.h`(画像サイズ。現状は`getinfo("image_w")`のみ)
- [ ] `obj.screen_w obj.screen_h`(直接変数としても提供)
- [ ] `obj.totaltime`(`time`と対。総時間秒)
- [ ] `obj.index obj.num`(個別オブジェクトの番号/総数。テキスト個別オブジェクトと連動)
- [ ] `obj.id`(オブジェクトID)
- [ ] `obj.frame/time/layer` の値の正しさ(`layer`は0固定、`frame`はEntity先頭起点かComp起点か要検証)
- [ ] `obj.track0-3`が実値でなくグローバル複製である点(値変更後の参照ずれ)

関数:
- [ ] `obj.getpixel(x,y[,"col"|"rgb"])`(引数ありで色取得。現状は常に(w,h)を返す)
- [ ] `obj.putpixel(x,y,r,g,b,a)` / `obj.copypixel(dx,dy,sx,sy)` / `obj.pixeloperation`
- [ ] `obj.getvalue(target[,time])`(`"x"`,`"y"`,`"zoom"`,`"rotation"`,`"track0"`等。他フレームの値取得)
- [ ] `obj.rand(min,max[,seed,frame])`(決定的乱数。anmスクリプトで頻出)
- [ ] `obj.interpolation(time,x0,y0,z0,x1,y1,z1,...)`(補間)
- [ ] `obj.load(type,...)`: `"image" "movie" "figure" "text" "frame" "layer" "framebuffer"`等(現状は警告のみ)
- [ ] `obj.setfont(name,size[,type,col1,col2])`(現状no-op。第2章と連動)
- [ ] `obj.setanchor(name,num[,option])`(ビューア上のドラッグ制御点。基点(cx/cy/cz)とは別機能)
- [ ] `obj.getoption`(`"camera_mode"`,`"multi_object"`等。現状no-op)
- [ ] `obj.setoption("camera_mode"|"blend"|"billboard"|"focus_mode"|"antialias"|"sampler"...)`(現状drawtargetのみ)
- [ ] `obj.draw`のrx/ry(3D回転)、zの反映、`drawpoly`のUV引数
- [ ] `obj.effect()`: 引数なし呼び出し(スクリプト側で設定済みの値を適用)、名前ゆらぎ(AviUtl内蔵名→movutl内蔵フィルタ名の対応表)、数値以外の引数
- [ ] `obj.getaudio(buf,file,type,size)` / `obj.filter` / `obj.clearbuffer` / `obj.mesh`
- [ ] `obj.getinfo`のAviUtl正規キー(`"clock" "saving" "script_path" "image_max" "version" "editing" "multi_object" "camera_mode" "camera"`。現状`image_w/image_h/...`は独自キー)

グローバル:
- [ ] `rand` `tostring2` 等のAviUtl独自グローバル、`RGB`の逆変換、`RGBtoHSV`/`HSVtoRGB`系(要確認)
- [ ] `require("mod")` / `package.path`のexedit script dir解決

## 4. exoインポート(`command/exo/exo_import.cpp`)の取りこぼし

test.exoの実データに現れるもの/AviUtl標準に存在するもの。

- [ ] `blend`値→`Entity::blend_`(通常/加算/減算/乗算/スクリーン/オーバーレイ/比較(明)/比較(暗)/輝度/色差/陰影/明暗/差分)。現状は全て通常
- [ ] `camera=0`(カメラ制御対象外)、`overlay`(現在のレイヤーと同時に表示)
- [ ] テキスト: `サイズ align type B I spacing_x spacing_y 表示速度 color2 monospace ...`(第2章)
- [ ] 音声: `左右`(パン)、`音量`のトラックバー値(`100.0,100.0,1`= 開始値,終了値,移動方式)→ AnimProps(現状は先頭値のみ)
- [ ] トラックバーの移動方式(直線/加減速/瞬間/中間点 等)と`拡大率/X/Y/回転/透明度`のアニメーション化(現状は静的な値のみ)
- [ ] 標準描画以外のエフェクト全般(`拡張描画`,`クリッピング`,`ぼかし`,`グロー`等)→ movutlフィルタへの対応表
- [ ] `[exedit]`の`width/height/rate/scale/audio_rate`をCompositionへ反映(現状`length`と同様に無視)
- [ ] オブジェクト種別: `シーン`, `フレームバッファ`, `カメラ制御`, `グループ制御`, `時間制御`, `アニメーション効果`(`.anm`スクリプト参照), `カスタムオブジェクト`, `フィルタオブジェクト`, `図形の背景`(type=0) 等
- [ ] `動画ファイル`の`動画ファイルと連携`(音声との連動)
- [ ] 音声波形表示・`音声ファイル`の`再生位置`(sec)/ループ再生の実挙動確認

## 5. 最初の一歩(Phase 1 の作業単位案)

1. `AviUtlObjContext`にEntity情報(基準座標・サイズ・layer・index/num・id)を持たせ、`obj.x/y/z/w/h/layer/id/index/num/totaltime/cz/aspect`を`setup_obj_table`で提供
2. `perform_draw`(`obj.draw`と暗黙draw)へ`cx/cy/cz`(基点)と`aspect`を反映 ← 第1章の基点実装の土台になる
3. `obj.getpixel(x,y)`/`putpixel`/`copypixel`/`rand`/`getvalue`/`interpolation`を追加
4. 各項目にdoctestを追加(`tests/aviutl_script_test.cpp`に追記)
5. 上記が固まってからEntityの`anchor_`Vec3化とrender反映(Phase 2)へ
