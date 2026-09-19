#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/core/vector.hpp>

namespace mu {

class Image;

// 文字の揃え位置(AviUtlの揃え0-8と同じ並び)。posが置かれる基点は、文字ブロックのこの位置になる
enum TextAlign : int32_t {
  TextAlign_LeftTop      = 0,
  TextAlign_CenterTop    = 1,
  TextAlign_RightTop     = 2,
  TextAlign_LeftMiddle   = 3,
  TextAlign_CenterMiddle = 4,
  TextAlign_RightMiddle  = 5,
  TextAlign_LeftBottom   = 6,
  TextAlign_CenterBottom = 7,
  TextAlign_RightBottom  = 8,
};

// 文字装飾(AviUtlのtype 0-4と同じ並び)。装飾の色はdeco_color_
enum TextDecoration : int32_t {
  TextDeco_Plain       = 0,
  TextDeco_Shadow      = 1,
  TextDeco_ShadowLight = 2,
  TextDeco_Outline     = 3,
  TextDeco_OutlineThin = 4,
};

class TextEntt final : public Entity {
private:
  std::string last_key_; // 再描画が必要かの判定用。img_を作った時のパラメータを連結した文字列
  int32_t pad_       = 0; // img_の四辺に足した装飾用の余白(px)。揃えの基準は余白を除いた文字ブロック

  void re_render_image();

public:
  TextEntt() = default;
  TextEntt(const char* path);
  ~TextEntt() = default;

  Ref<Image> img_;
  int32_t dirty_ = 0;                                // MPROPERTY(name="更新フラグ", hidden=true)
  float speed    = 100.0;                            // MPROPERTY(name="再生速度")
  std::string font;                                  // MPROPERTY(name="フォント", type="font")
  std::string text;                                  // MPROPERTY(name="テキスト")
  bool separate         = false;                     // MPROPERTY(name="個別オブジェクト")
  int32_t font_size_    = 34;                        // MPROPERTY(name="サイズ", min=1, max=1000)
  bool bold_            = false;                     // MPROPERTY(name="太字")
  bool italic_          = false;                     // MPROPERTY(name="斜体")
  int32_t spacing_x_    = 0;                         // MPROPERTY(name="字間")
  int32_t spacing_y_    = 0;                         // MPROPERTY(name="行間")
  bool monospace_       = false;                     // MPROPERTY(name="等間隔")
  int32_t align_        = TextAlign_CenterMiddle;    // MPROPERTY(name="揃え(0-8: 左上,中央上,右上,左中,中央,右中,左下,中央下,右下)", hidden_inspector=true)
  int32_t deco_         = TextDeco_Plain;            // MPROPERTY(name="文字装飾(0:標準 1:影 2:影(薄) 3:縁取り 4:縁取り(細))", hidden_inspector=true)
  Vec4b color_          = Vec4b(255, 255, 255, 255); // MPROPERTY(name="文字色")
  Vec4b deco_color_     = Vec4b(0, 0, 0, 255);       // MPROPERTY(name="装飾色")

  // 揃え位置の基点が、画像中心からどれだけずれているか(px)。Entity::compositeのorigin_offsetに渡す
  Vec2 align_origin_offset() const;

  static Ref<TextEntt> Create(const char* text, const char* font = nullptr);
  virtual EntityType getType() const override { return EntityType_3DText; }
  virtual bool render(Composition* cmp, Image* target, int frame) override;

  virtual const cutil::PropInfo* getPropsInfo() const override; // MUFUNC_AUTOGEN
  virtual cutil::Prop getProps() const override;                // MUFUNC_AUTOGEN
  virtual void setProps(const cutil::Prop& props) override;     // MUFUNC_AUTOGEN
};

} // namespace mu
