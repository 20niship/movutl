#pragma once
#include <movutl/asset/entity.hpp>
#include <movutl/asset/image.hpp>

namespace mu {

class Movie final : public Entity {
public:
  Movie() = default;
  Movie(const char* path);
  ~Movie() = default;

  Ref<Image> img_;
  Vec3 pos = Vec3(0, 0, 0);    // MPROPERTY(name="位置", viewer_anchor=true)
  Vec2 scale = Vec2(100, 100); // MPROPERTY(name="拡大率", scale=true)
  float rotation = 0;          // MPROPERTY(name="回転")
  int start_frame_ = 0;        // MPROPERTY(name="開始フレーム")
  float speed = 100.0;         // MPROPERTY(name="再生速度", min=0.0, max=10000.0, step=5.0)
  uint8_t alpha_ = 255;        // MPROPERTY(name="透明度")
  bool loop_ = false;          // MPROPERTY(name="ループ再生")
  bool with_alpha_ = false;    // MPROPERTY(name="透明度を読み込む")
  std::string path_;           // MPROPERTY(name="ファイル", type="path")

  static Ref<Movie> Create(const char* name, const char* path = nullptr);
  bool load_file(const char* path);
  virtual EntityType getType() const override { return EntityType_Movie; }
  virtual bool render(Composition* cmp) override;

  virtual PropsInfo getPropsInfo() const override;    // MUFUNC_AUTOGEN
  virtual Props getProps() const override;            // MUFUNC_AUTOGEN
  virtual void setProps(const Props& props) override; // MUFUNC_AUTOGEN
};

} // namespace mu
