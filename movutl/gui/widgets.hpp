#pragma once
#include <array>
#include <cstdint>
#include <movutl/core/vector.hpp>

namespace mu {
class Entity;
struct AnimProps;

void wd_entt_props_editor(Entity* e, uint32_t cur_frame);
void wd_movie_inspector(Entity* e);
void wd_image_inspector(Entity* e);
bool wd_color_edit(const char* name, Vec4b* col);

// キーフレームトグルボタン(cur_frameに有れば削除、無ければ追加)。戻り値trueなら変化(要キャッシュ無効化)
bool wd_keyframe_toggle(AnimProps& anim, int idx, uint32_t cur_frame);

// ミニタイムラインstrip([fstart,fend]、ドラッグ移動/右クリック削除、ダブルクリックでイージング編集ポップアップ)。戻り値trueなら変化(要キャッシュ無効化)
bool wd_keyframe_strip(const char* str_id, AnimProps& anim, int idx, int fstart, int fend, uint32_t cur_frame);

// cubic-bezier(v[0],v[1],v[2],v[3])のハンドルをsize四方の正方形プレビュー上でドラッグ編集する。戻り値trueなら変化
bool wd_bezier_handle_editor(std::array<float, 4>& v, float size);

} // namespace mu
