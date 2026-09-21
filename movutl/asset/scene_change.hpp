#pragma once
#include <movutl/asset/entity.hpp>
#include <string>

namespace mu {

enum SceneChangeType {
  SceneChangeType_Fade   = 0, // フェード/クロスフェード
  SceneChangeType_WipeLR = 1, // 左右ワイプ
  SceneChangeType_WipeUD = 2, // 上下ワイプ
  SceneChangeType_Circle = 3, // 円形ワイプ(中心から外へ)
  SceneChangeType_Count  = 4,
};

// 進行度p(0..1)。frame=fstartで0、fendで1。区間長0以下なら1
float SceneChangeProgress(int frame, int fstart, int fend);
// 遷移後(incoming)の重み(0..1)。(nx,ny)は画像内の正規化座標(0..1)。blurは境界のぼかし幅(0..1)、invertは方向反転
float SceneChangeWeight(int type, float p, float nx, float ny, bool invert, float blur);
// exoのシーンチェンジ名(「フェード」「左から右へワイプ」等)から種類/反転を決める。未知の名前はfalse
bool SceneChangeFromExoName(const std::string& name, int& type, bool& invert);

// AviUtlのシーンチェンジ相当。自身より1つ上(layer+1)のレイヤーで、直前のオブジェクトAから区間内で始まるオブジェクトBへ進行度pで切り替える
// (docs: 区間開始=切替位置、Aは終端フレームを保持して区間中も描画される)。自身は描画せず、CPURenderer::render_frameが処理する
class SceneChangeEntt final : public Entity {
public:
  SceneChangeEntt()  = default;
  ~SceneChangeEntt() = default;

  int32_t type_ = SceneChangeType_Fade; // MPROPERTY(name="種類(0:フェード 1:左右ワイプ 2:上下ワイプ 3:円形)", min=0, max=3)
  bool invert_  = false;                // MPROPERTY(name="反転", desc="ワイプの方向を反転する")
  float blur_   = 0.0f;                 // MPROPERTY(name="ぼかし", desc="ワイプ境界のぼかし幅(0〜1)", min=0.0, max=1.0, step=0.01)

  static Ref<SceneChangeEntt> Create(const char* name);
  virtual EntityType getType() const override { return EntityType_SceneChange; }
  virtual bool render(Composition*, Image*, int) override { return true; }

  float progress(int frame) const { return SceneChangeProgress(frame, fstart_, fend_); }
  // scene_layerにあるこのシーンチェンジが、layer_iのEntityへ効くか(直上の1レイヤーのみ)
  bool affects(int scene_layer, int layer_i) const { return layer_i == scene_layer + 1; }

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
