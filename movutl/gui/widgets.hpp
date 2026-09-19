#pragma once
#include <movutl/core/vector.hpp>

namespace mu {
class Entity;

// Entity固有プロパティ(トランスフォーム以外)を「ラベル左・値右」の表で編集する
void wd_entt_props_editor(Entity* e);
// トランスフォーム(位置/基点/拡大率/回転/不透明度)と基点プリセットを編集する。描画変換を持たないEntityでは何もしない
void wd_entt_transform_editor(Entity* e);
void wd_movie_inspector(Entity* e);
void wd_image_inspector(Entity* e);
bool wd_color_edit(const char* name, Vec4b* col);

// 「ラベル左・値右」の2列表。wd_table_begin()がtrueのときだけwd_row()/wd_table_end()を呼ぶ。wd_row()後の次のウィジェットは幅-FLT_MINで値列いっぱいに広がる(ラベルは"##id"で隠すこと)
bool wd_table_begin(const char* id);
void wd_table_end();
void wd_row(const char* label, const char* desc = "");
// 3x3の位置選択グリッド(左上〜右下、ミニ矩形アイコン)。selectedは0-8か-1。クリックされたらtrueを返し*pickedに0-8を入れる。tipsは9個のツールチップ(任意)
bool wd_grid9(const char* id, int selected, int* picked, const char* const* tips = nullptr);

} // namespace mu
