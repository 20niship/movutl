#pragma once
#include <array>
#include <cstdint>
#include <cutil/prop.hpp>
#include <movutl/core/vector.hpp>

namespace mu {
class Entity;
struct AnimProps;

// Entity固有プロパティ(トランスフォーム以外)を編集する。アニメーション可能なプロパティはトラックバー行、それ以外は「ラベル左・値右」の表。cur_frameはコンポジション上の現在フレーム
void wd_entt_props_editor(Entity* e, uint32_t cur_frame);
// トランスフォーム(位置/基点/拡大率/回転/不透明度)と基点プリセットを同様に編集する。描画変換を持たないEntityでは何もしない
void wd_entt_transform_editor(Entity* e, uint32_t cur_frame);
void wd_movie_inspector(Entity* e);
void wd_image_inspector(Entity* e);
bool wd_color_edit(const char* name, Vec4b* col);

// 「ラベル左・値右」の2列表。wd_table_begin()がtrueのときだけwd_row()/wd_table_end()を呼ぶ。wd_row()後の次のウィジェットは幅-FLT_MINで値列いっぱいに広がる(ラベルは"##id"で隠すこと)
bool wd_table_begin(const char* id);
void wd_table_end();
void wd_row(const char* label, const char* desc = "");
// 3x3の位置選択グリッド(左上〜右下、ミニ矩形アイコン)。selectedは0-8か-1。クリックされたらtrueを返し*pickedに0-8を入れる。tipsは9個のツールチップ(任意)
bool wd_grid9(const char* id, int selected, int* picked, const char* const* tips = nullptr);

// AviUtl風1行トラックバー(左=直前中間点値/中央=名前ボタン/右=直後中間点値)。cur_frameはトラック開始からの相対frame、lengthはトラック長(未アニメ時の区間アニメ終点)。filter_index=-1はEntity本体のanim_props_
bool wd_animatable_row(const cutil::PropInfo::Field& f, AnimProps& anim, int idx, uint32_t cur_frame, uint64_t entity_guid, int filter_index, int length);

// インスペクタ最上部の集約バー(中間点分布+現在フレーム、クリック/ドラッグでシーク)。戻り値trueならシークが発生した
bool wd_entity_keyframe_overview(Entity* e, uint32_t cur_frame);

// cubic-bezier(v[0],v[1],v[2],v[3])のハンドルをsize四方の正方形プレビュー上でドラッグ編集する。戻り値trueなら変化
bool wd_bezier_handle_editor(std::array<float, 4>& v, float size);

} // namespace mu
