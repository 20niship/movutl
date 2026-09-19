#pragma once
#include <movutl/asset/entity.hpp>

namespace mu {

// AviUtlのグループ制御相当。自身は描画せず、下のレイヤーのEntityへ自身の変換を親として与える(docs/座標系と基点の仕様.md §2.5)
class GroupEntt final : public Entity {
public:
  GroupEntt()  = default;
  ~GroupEntt() = default;

  int32_t target_layers_ = 0; // MPROPERTY(name="対象レイヤー数", desc="自身より下の何レイヤーに効かせるか。0なら以降すべて", min=0)

  static Ref<GroupEntt> Create(const char* name);
  virtual EntityType getType() const override { return EntityType_Group; }
  virtual bool render(Composition*, Image*, int) override { return true; }

  // このグループの局所変換(親として子へ渡す前の値)
  GroupXform local_xform() const { return {pos_, scale_, rotation_, alpha_}; }
  // group_layerにあるこのグループが、layer_iのEntityへ効くか
  bool affects(int group_layer, int layer_i) const { return layer_i > group_layer && (target_layers_ <= 0 || layer_i <= group_layer + target_layers_); }

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
