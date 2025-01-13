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
  int start_frame_ = 0;     // MPROPERTY(name="開始フレーム")
  float speed = 100.0f;     // MPROPERTY(name="再生速度")
  uint8_t alpha_ = 255;     // MPROPERTY(name="透明度")
  bool loop_ = false;       // MPROPERTY(name="ループ再生")
  bool with_alpha_ = false; // MPROPERTY(name="透明度を読み込む")
  std::string path_;        // MPROPERTY(name="ファイル", type="path")

  static Ref<Movie> Create(const char* name, const char* path = nullptr);
  bool load_file(const char* path);
  virtual EntityType getType() const override { return EntityType_Movie; }
  virtual bool render(Composition* cmp) override;

  virtual PropsInfo getPropsInfo() const override;    // MUFUNC_AUTOGEN
  virtual Props getProps() const override;            // MUFUNC_AUTOGEN
  virtual void setProps(const Props& props) override; // MUFUNC_AUTOGEN
};

} // namespace mu
