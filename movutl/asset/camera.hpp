#pragma once

#include <movutl/asset/entity.hpp>

namespace mu {

inline constexpr float kCameraDefaultZ = -1024.0f; // AviUtl既定のカメラZ(Z=0面が等倍になる距離)

// AviUtlのカメラ制御相当。自身は描画せず、自身より下のレイヤーの「カメラ制御」ONのEntityへ視点変換を親として与える。
// カメラ位置X/Y/Zは pos_、傾きは rotation_ を流用する(AviUtl既定はZ=-1024でスケール等倍)
class Camera3D final : public Entity {
public:
  Camera3D() { pos_ = Vec3(0, 0, kCameraDefaultZ); }
  ~Camera3D() = default;

  Vec3 target_           = Vec3(0, 0, 0); // MPROPERTY(name="目標位置", desc="カメラが向く点(2D近似では未使用)")
  float fov_             = 45.0f;         // MPROPERTY(name="視野角(度)", desc="2D近似では未使用", min=1.0, max=170.0)
  int32_t target_layers_ = 0;             // MPROPERTY(name="対象レイヤー数", desc="自身より下の何レイヤーに効かせるか。0なら以降すべて", min=0)

  static Ref<Camera3D> Create(const char* name);
  virtual EntityType getType() const override { return EntityType_Camera; }
  virtual bool render(Composition*, Image*, int) override { return true; }

  // camera_layerにあるこのカメラが、layer_iのEntityへ効くか
  bool affects(int camera_layer, int layer_i) const { return layer_i > camera_layer && (target_layers_ <= 0 || layer_i <= camera_layer + target_layers_); }

  // 世界座標→画面座標の視点変換(2D近似: カメラXYの逆平行移動、Zによる遠近スケール、傾きの逆回転)。カメラがZ=0面の前後(距離<1)にある場合は等倍
  // ponytail: 目標位置/視野角によるパン・チルト・ズームと深度ボケは無し(CPURendererは2D合成のみ)。3D対応時にrot_x_/rot_y_へ反映する
  GroupXform view_xform() const;

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
