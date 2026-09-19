#pragma once
#include <movutl/asset/composition.hpp>
#include <movutl/asset/entity.hpp>
#include <movutl/gui/transform_gizmo.hpp>

namespace mu {

// Entityの現在の描画変換と、コンポ上での表示枠(ギズモ・ヒットテスト用)
struct EntityGizmo {
  GizmoXform xform;
  GizmoPt src_size;
  GizmoPt origin_offset;
  GizmoPt comp_size;
  GizmoQuad quad;
  GizmoPt anchor_pt;
};

// 変換を持つEntityのギズモ情報を求める。画像サイズが不明(CompoRef等)ならコンポ全体を枠とみなす。変換を持たなければfalse
bool entity_gizmo_of(const Entity& e, const GizmoPt& comp_size, EntityGizmo& out);
// xformをEntityのpos_/anchor_/scale_/rotation_へ書き戻す(pos_/anchor_のZは維持)
void entity_apply_xform(Entity& e, const GizmoXform& x);

// comp_pt(コンポ左上原点px)を含む最前面の可視Entityを返す(描画順の後ろほど前面)。無ければnull
Ref<Entity> hit_test_entity(const Composition& cmp, const GizmoPt& comp_pt);

} // namespace mu
