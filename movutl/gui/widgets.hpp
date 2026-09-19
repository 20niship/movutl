#pragma once
#include <array>
#include <cstdint>
#include <cutil/prop.hpp>
#include <movutl/core/vector.hpp>

namespace mu {
class Entity;
struct AnimProps;

void wd_entt_props_editor(Entity* e, uint32_t cur_frame);
void wd_movie_inspector(Entity* e);
void wd_image_inspector(Entity* e);
bool wd_color_edit(const char* name, Vec4b* col);

// AviUtl風1行トラックバー(左=直前中間点値/中央=名前ボタン/右=直後中間点値)。cur_frameはトラック開始からの相対frame、lengthはトラック長(未アニメ時の区間アニメ終点)。filter_index=-1はEntity本体のanim_props_
bool wd_animatable_row(const cutil::PropInfo::Field& f, AnimProps& anim, int idx, uint32_t cur_frame, uint64_t entity_guid, int filter_index, int length);

// インスペクタ最上部の集約バー(中間点分布+現在フレーム、クリック/ドラッグでシーク)。戻り値trueならシークが発生した
bool wd_entity_keyframe_overview(Entity* e, uint32_t cur_frame);

// cubic-bezier(v[0],v[1],v[2],v[3])のハンドルをsize四方の正方形プレビュー上でドラッグ編集する。戻り値trueなら変化
bool wd_bezier_handle_editor(std::array<float, 4>& v, float size);

} // namespace mu
